"""
gen_sec.py

Create salt, verifier datablocks for given username/password
Default username is Clock_Monsieur
Default password is 4 chars based on (epoch time / 10)
Specify partition to flash data into that partition
    else it just prints the data
"""

import os
import re
import subprocess
import sys
import argparse
import time
from pathlib import Path
from io import StringIO
import qrcode
from qr_code_pdf import create_qr_pdf

# --- CONFIGURATION & CONSTANTS ---
PASSWORD_CHAR_SET = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
PASSWORD_LENGTH = 4
DEFAULT_USERNAME = "Clock_Monsieur"
DEFAULT_PARTITION_NAME = "prov_dat"
DEFAULT_SERVICE_NAME = "Clock Monsieur"
TEMP_BINARY_FILENAME = "prov_creds.bin"
PARTITION_TABLE_REL_PATH = Path("partition_table") / "partition-table.bin"
PARTITION_TOOL_REL_PATH = Path("components") / "partition_table" / "gen_esp32part.py"

# Storage sizes (must match C structure)
PASSWORD_STORAGE_SIZE = 15
SRP6A_SALT_LEN = 16
SRP6A_VERIFIER_LEN = 384

"""
struct sec_info_t {
    uint8_t password_len;
    char password[15];
    char salt[16];
    char verifier[384];
};
"""

idf_path = Path("")
build_dir = Path("")


def esp_tool(arguments):
    """Executes the esptool.py command, returns stdout"""
    environment = os.environ.copy()
    environment['IDF_PATH'] = str(idf_path)
    tool = idf_path / "components" / "esptool_py" / "esptool" / "esptool.py"
    command = [sys.executable, str(tool), "--port", args.port, "--chip", args.chip] + arguments
    try:
        result = subprocess.run(command, check=True, text=True, capture_output=True, env=environment)
        time.sleep(1)
        return result.stdout
    except subprocess.CalledProcessError as err:
        print(f"ERROR: esptool failed with exit code {err.returncode}.")
        print(f"STDOUT:\n{err.stdout}")
        print(f"STDERR:\n{err.stderr}")
        return None
    except FileNotFoundError:
        print("ERROR: esptool.py not found. Ensure your ESP-IDF environment is active.")
        return None


def get_mac_address():
    # MAC: 10:20:ba:1a:00:f8
    x = esp_tool(["read_mac"])
    if not x:
        raise Exception("Failed to get MAC address from esptool.py")
    lines = StringIO(x)
    for line in lines:
        stripped = line.strip()
        if line.startswith("MAC:"):
            print(f'MAC Address found : {line.strip()}')
            mac_address = int("".join(line[4:].split(":")), base=16)
            print(f'MAC Address parsed: {mac_address:012x}')
            return mac_address
    return 0


def generate_mac_based_password():
    """
    Generate a password from low 21 bits of device mac address
    """
    mac = get_mac_address() & ((1 << 20) - 1)  # get low 20 bits of mac address
    base = len(PASSWORD_CHAR_SET)
    new_password = ""
    while mac > 0:
        new_password = f"{PASSWORD_CHAR_SET[mac % base]}{new_password}"
        mac //= base
    return new_password.zfill(PASSWORD_LENGTH)


def validate_custom_password(custom_password):
    """
    Ensure that user-supplied password is a valid base 36 number
    :param custom_password
    :return: validated password
    """
    if len(custom_password) != PASSWORD_LENGTH:
        raise ValueError(f"Password must be exactly {PASSWORD_LENGTH} characters long.")
    for char in custom_password.upper():
        if char not in PASSWORD_CHAR_SET:
            raise ValueError(
                f"Password contains invalid character '{char}'. Only A-Z and 0-9 are allowed."
            )
    return custom_password.upper()


def get_sec_output(username, actual_password):
    """
    Executes esp_prov.py to get Salt and Verifier, returns (salt_hex, verifier_hex)
    :param username:
    :param actual_password:
    :return: stdout from esp_prov.py
    """
    script_path = idf_path / "tools" / "esp_prov" / "esp_prov.py"

    command = [
        sys.executable,
        str(script_path),
        "--transport",
        "softap",
        "--sec_ver",
        "2",
        "--sec2_gen_cred",
        "--sec2_username",
        username,
        "--sec2_pwd",
        actual_password,
    ]

    env = os.environ.copy()
    env['IDF_PATH'] = str(idf_path)

    try:
        return subprocess.run(command, capture_output=True, text=True, check=True, encoding="utf-8", env=env).stdout
    except subprocess.CalledProcessError as err:
        print(f"Error: esp_prov.py failed with return code {err.returncode}")
        print(f"Stdout:\n{err.stdout}")
        print(f"Stderr:\n{err.stderr}")
        return None
    except FileNotFoundError:
        print("Error: Python executable or esp_prov.py not found. Is IDF_PATH sourced?")
        return None


def parse_sec_output(output):
    # Parse the output
    salt_match = re.search(r"sec2_salt\[] = \{(.*?)}", output, re.DOTALL)
    verifier_match = re.search(r"sec2_verifier\[] = \{(.*?)}", output, re.DOTALL)

    if not salt_match or not verifier_match:
        print("Error: Failed to parse Salt or Verifier from script output.")
        return None, None

    def clean_hex(match_group):
        return (
            match_group.group(1)
            .replace("0x", "")
            .replace(",", "")
            .replace(" ", "")
            .replace('\r', '')
            .replace('\n', '')
            .strip()
        )

    salt = clean_hex(salt_match)
    verifier = clean_hex(verifier_match)

    return salt, verifier


def create_unified_binary(actual_password, salt, verifier, output_path):
    """Generates the unified binary file containing password, salt, and verifier."""

    # 1. Construct the Unified Binary Data
    binary_data = bytearray()

    # a. Password data (Length + Padded String)
    binary_data.append(len(actual_password))
    password_bytes = actual_password.encode("ascii")
    binary_data.extend(password_bytes)
    binary_data.extend(
        b"\x00" * (PASSWORD_STORAGE_SIZE - len(password_bytes))
    )  # Padding

    # b. Salt and Verifier
    try:
        binary_data.extend(bytes.fromhex(salt))
        binary_data.extend(bytes.fromhex(verifier))
    except ValueError:
        print("Error: Invalid hexadecimal characters found during conversion.")
        return None, None

    # Final check on size
    expected_size = 1 + PASSWORD_STORAGE_SIZE + SRP6A_SALT_LEN + SRP6A_VERIFIER_LEN
    if len(binary_data) != expected_size:
        print(
            f"Internal Error: Binary size mismatch. Expected {expected_size}, got {len(binary_data)}"
        )
        return None, None

    # 2. Write the final unified binary file
    try:
        with open(output_path, "wb") as f:
            f.write(binary_data)
        return output_path

    except IOError as err:
        print(f"Error: Could not write binary file to {output_path}. Reason: {err}")
        return None


# --- FLASHING FUNCTIONS ---


def find_partition_offset_and_size(partition_name):
    """
    Reads the partition-table.bin file and uses gen_esp32part.py to dump its contents
    for parsing the starting offset of the specified partition.
    """
    bin_path = build_dir / PARTITION_TABLE_REL_PATH
    tool_path = idf_path / PARTITION_TOOL_REL_PATH

    if not bin_path.exists():
        print(f"Error: Partition table binary not found at {bin_path}.")
        print("Please ensure your project has been built (idf.py build).")
        return None, None

    if not tool_path.exists():
        print(f"Error: Partition generation tool not found at {tool_path}.")
        return None, None

    # print(f"Dumping partition table from binary using: {tool_path.name}")

    # CORRECTED COMMAND: Pass the binary file path directly without the '--dump' flag
    command = [sys.executable, tool_path, bin_path]

    try:
        # Execute the dump command
        result = subprocess.run(
            command, capture_output=True, text=True, check=True, encoding="utf-8"
        )
        csv_data = result.stdout
    except subprocess.CalledProcessError as err:
        print(f"Error executing gen_esp32part.py: {err.stderr}")
        return None, None

    # Use StringIO to treat the output string as a file object for line parsing
    csv_file = StringIO(csv_data)

    for line in csv_file:
        # Skip header/comment lines (which start with # or 'Parsing'/'Verifying')
        if (
                line.startswith("#")
                or line.startswith("Parsing")
                or line.startswith("Verifying")
                or not line.strip()
        ):
            continue

        # Split the line by comma. The output you provided is clean.
        parts = [part.strip() for part in line.split(",")]

        # Check if the name matches the target (prov_dat) and has enough parts
        if len(parts) >= 4 and parts[0] == partition_name:
            # The fourth element (index 3) is the Offset
            offset_hex = parts[3].strip()

            # decode size
            # it can be
            # just an int
            # a hex number 0x.....
            # 24K
            # 2M

            multiplier = 1
            line_size = parts[4].strip()
            if line_size.endswith("K"):
                multiplier = 1024
                line_size = line_size[:-1]
            elif line_size.endswith("M"):
                multiplier = 1024 * 1024
                line_size = line_size[:-1]
            if line_size.startswith("0x"):
                base = 16
                line_size = line_size[2:]
            else:
                base = 10
            byte_size = int(line_size, base) * multiplier

            # Since the output is consistently '0x...', we can rely on that format.
            if re.match(r"0x[0-9a-fA-F]+$", offset_hex):
                print(f"Found partition '{partition_name}' at offset: {offset_hex}")
                return offset_hex, byte_size

            # Catch case where size (5th element) might be mistaken for offset if parsing goes wrong
            # Your specific output showed the correct offset: '0x11000'
            print(f"Error parsing offset for '{partition_name}': {offset_hex}")
            return None, None

    print(f"Error: Partition '{partition_name}' not found in the dumped table data.")
    return None, None


def execute_flash_command(offset_hex, binary_file):
    """flash the binary file"""
    print(f"Flashing {binary_file.name} to {args.chip} on {args.port} at offset {offset_hex}...")
    return esp_tool(["write_flash", offset_hex, str(binary_file)])


def clear_partition(name):
    partition_offset, partition_size = find_partition_offset_and_size(name)
    print(f'Erasing partition {name} at offset {partition_offset}, size = {partition_size}')
    return esp_tool(['erase_region', partition_offset, str(partition_size)])


def create_qr_code(data: str, filename: Path, box_size: int = 10, border: int = 4,
                   error_correction_level=qrcode.ERROR_CORRECT_L):
    """
    Encodes a string as a QR code and saves it as a PNG file.

    Args:
        data (str): The string content to encode in the QR code.
        filename (str): The name of the output file (e.g., 'my_qrcode.png').
        box_size (int): Controls the size of each box (pixel) in the QR code.
                        A higher number means a larger image. (Default: 10)
        border (int): Controls the width of the white border around the code.
                      (Default: 4 is the spec minimum)
        error_correction_level (int): The level of error correction.
                                      L (Low, 7%), M (Medium, 15%), Q (Quartile, 25%), H (High, 30%).
                                      Uses L by default.
    """
    try:
        qr = qrcode.QRCode(
            version=1,  # 1 is the smallest version, higher for more data
            error_correction=error_correction_level,
            box_size=box_size,
            border=border,
        )
        qr.add_data(data)
        qr.make(fit=True)
        img = qr.make_image(fill_color="black", back_color="white")
        img.save(filename)
        print(f"QR code saved to {filename}")

    except Exception as ex:
        print(f"Error creating QR code: {ex}")


def make_qr_code():
    """
    ver = v1
    name = service_name
    username = username
    pop = password
    transport = ble
    """
    ver = 'v1'
    name = args.service_name
    username = args.username
    pop = password
    transport = 'ble'
    payload = f'{{"ver":"{ver}","name":"{name}","username":"{username}","pop":"{pop}","transport":"{transport}"}}'
    filename = build_dir / "qr_code.png"
    print(f'Create QR code at {filename}, payload {payload}')
    create_qr_code(payload, filename, box_size=8, border=2)
    create_qr_pdf(filename, build_dir / "qr_code_label.pdf", "APP", '"ESP BLE<br/>Provisioning"', f'PIN:  {password}')


# --- MAIN EXECUTION ---

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Generate SRP6a credentials and flash to the custom partition.",
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("--build_dir", help="Project build directory", required=True, type=str, metavar='dir')
    parser.add_argument("--port", help="Serial port of the device (e.g., COM3 or /dev/ttyUSB0).", metavar='port')
    parser.add_argument("--chip", help="Target ESP chip type (e.g., esp32, esp32s3).", metavar='chip')
    parser.add_argument("--idf_path", help="Override env $IDF_PATH", metavar='dir')
    parser.add_argument("--username", help=f'Username', default=DEFAULT_USERNAME, metavar='name')
    parser.add_argument("--password", help="Optional custom password (4 chars, A-Z/0-9) for testing.",
                        metavar='password')
    parser.add_argument('--service_name', help=f'BLE service name', default=DEFAULT_SERVICE_NAME, metavar='name')
    parser.add_argument("--flash", help="Flash partition with security data.", action="store_true")
    parser.add_argument("--partition", help=f'SecInfo partition', default=DEFAULT_PARTITION_NAME, metavar='name')
    parser.add_argument('--clear_nvs', help="Clear NVS partition.", action="store_true")
    parser.add_argument('--clear_sec_info', help="Clear SecInfo partition.", action="store_true")

    # --build_dir ../firmware/build --port COM4 --chip esp32s3 --clear_nvs --flash

    args = parser.parse_args()

    try:
        if args.idf_path:
            idf_path = Path(args.idf_path)
        else:
            idf_path_env = os.getenv("IDF_PATH", None)
            if not idf_path_env:
                raise Exception("IDF_PATH environment variable not set.")
            idf_path = Path(idf_path_env)

        need_port_etc = args.flash or args.clear_nvs or args.clear_sec_info

        if need_port_etc and not (args.port and args.chip):
            raise Exception("Need --port and --chip to flash")

        build_dir = Path(args.build_dir)

        # Password
        if args.password:
            password = validate_custom_password(args.password)
            print("--- USING CUSTOM PASSWORD FOR TESTING ---")
        else:
            password = generate_mac_based_password()
            print("--- USING MAC ADDRESS PASSWORD (PRODUCTION MODE) ---")

        print(f"Provisioning Password: {password}")

        # Get salt, verifier output
        sec_output = get_sec_output(args.username, password)
        if not sec_output:
            raise Exception("Failed to get sec output")

        print(sec_output)

        make_qr_code()

        if args.clear_nvs:
            clear_partition('nvs')

        if args.clear_sec_info:
            clear_partition(args.partition)

        if not args.flash:
            sys.exit(0)

        # Parse into binary
        salt_hex, verifier_hex = parse_sec_output(sec_output)

        if not (salt_hex and verifier_hex):
            raise Exception("Failed to parse sec output")

        # Create Unified Binary File
        output_dir = build_dir / "temp_prov"
        output_dir.mkdir(parents=True, exist_ok=True)

        binary_path = output_dir / TEMP_BINARY_FILENAME

        print(f"Generating unified binary file: {binary_path}...")
        final_binary_path = create_unified_binary(
            password, salt_hex, verifier_hex, binary_path
        )

        if not final_binary_path:
            raise Exception("Failed to create provisioning binary.")

        # Find partition offset for flashing
        offset, size = find_partition_offset_and_size(args.partition)

        if not offset:
            raise Exception("Could not determine flash offset from build directory.")

        # Flash partition
        if not execute_flash_command(offset, final_binary_path):
            raise Exception("PROVISIONING FAILED!")

        print("Flashed successfully")
        print(f"User provisioning password: {password}")
        print(f"User provisioning username: {args.username}")

    except Exception as e:
        print(e, file=sys.stderr)
        sys.exit(1)
