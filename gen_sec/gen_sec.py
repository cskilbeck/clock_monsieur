"""
gen_sec.py

Create salt, verifier datablocks for given username/password
Default username is Clock_Monsieur
Default password is 4 chars based on (epoch time / 10)
Specify partition_name to flash data into that partition
    else it just prints the data
"""

import os
import re
import subprocess
import sys
import argparse
from pathlib import Path
from io import StringIO

# --- CONFIGURATION & CONSTANTS ---
PASSWORD_CHAR_SET = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
PASSWORD_LENGTH = 4
DEFAULT_USERNAME = "Clock_Monsieur"
DEFAULT_PARTITION_NAME = "prov_dat"
TEMP_BINARY_FILENAME = "prov_creds.bin"
PARTITION_TABLE_REL_PATH = Path("partition_table") / "partition-table.bin"
PARTITION_TOOL_REL_PATH = Path("components") / "partition_table" / "gen_esp32part.py"

# Storage sizes (must match C structure)
PASSWORD_STORAGE_SIZE = 16
SRP6A_SALT_LEN = 16
SRP6A_VERIFIER_LEN = 96

idf_path = Path("")
build_dir = Path("")

"""
struct sec_info_t {
    char password[4];
    char salt[16];
    char verifier[xxx];
};
"""


def esp_tool(arguments):
    """Executes the esptool.py command, returns stdout"""
    tool = idf_path / 'components' / 'esptool_py' / 'esptool.py'
    command = ['python', tool, "--port", args.port] + arguments
    try:
        result = subprocess.run(command, check=True, text=True, capture_output=True)
        return result.stdout
    except subprocess.CalledProcessError as err:
        print(f'ERROR: esptool failed with exit code {err.returncode}.')
        print(f"STDOUT:\n{err.stdout}")
        print(f"STDERR:\n{err.stderr}")
        return None
    except FileNotFoundError:
        print('ERROR: esptool.py not found. Ensure your ESP-IDF environment is active.')
        return None


def get_mac_address_24():
    # MAC: 10:20:ba:1a:00:f8
    x = esp_tool(['read_mac'])
    lines = StringIO(x)
    for line in lines:
        if line.strip().startswith('MAC:'):
            return int("".join(line[4:].strip().split(':')[3:]), base=16)
    return 0


def generate_mac_based_password():
    """
    Generate a password from low 21 bits of device mac address
    """
    mac = get_mac_address_24() & ((1 << 21) - 1)  # get low 21 bits of mac address
    base = len(PASSWORD_CHAR_SET)
    new_password = ""
    while mac > 0:
        new_password = f'{PASSWORD_CHAR_SET[mac % base]}{new_password}'
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
            raise ValueError(f"Password contains invalid character '{char}'. Only A-Z and 0-9 are allowed.")
    return custom_password.upper()


def get_sec_output(username, actual_password):
    """
    Executes esp_prov.py to get Salt and Verifier, returns (salt_hex, verifier_hex)
    :param username:
    :param actual_password:
    :return: stdout from esp_prov.py
    """
    script_path = idf_path / "tools" / "esp_prov" / "esp_prov.py"

    command = ["python", script_path,
               "--transport", "softap",
               "--sec_ver", "2",
               "--sec2_gen_cred",
               "--sec2_username", username,
               "--sec2_pwd", actual_password]

    try:
        return subprocess.run(command, capture_output=True, text=True, check=True, encoding="utf-8").stdout
    except subprocess.CalledProcessError as err:
        print(f"Error: esp_prov.py failed with return code {err.returncode}")
        print(f"Stdout:\n{err.stdout}")
        print(f"Stderr:\n{err.stderr}")
        return None
    except FileNotFoundError:
        print('Error: Python executable or esp_prov.py not found. Is IDF_PATH sourced?')
        return None


def parse_sec_output(output):
    # Parse the output
    salt_match = re.search(r"sec2_salt\[] = \{(.*?)}", output, re.DOTALL)
    verifier_match = re.search(r"sec2_verifier\[] = \{(.*?)}", output, re.DOTALL)

    if not salt_match or not verifier_match:
        print('Error: Failed to parse Salt or Verifier from script output.')
        return None, None

    def clean_hex(match_group):
        return (
            match_group.group(1)
            .replace("0x", "")
            .replace(",", "")
            .replace(" ", "")
            .strip()
        )

    return clean_hex(salt_match), clean_hex(verifier_match)


def create_unified_binary(actual_password, salt, verifier, output_path):
    """Generates the unified binary file containing password, salt, and verifier."""

    # 1. Construct the Unified Binary Data
    binary_data = bytearray()

    # a. Password data (Length + Padded String)
    binary_data.append(len(actual_password))
    password_bytes = actual_password.encode("ascii")
    binary_data.extend(password_bytes)
    binary_data.extend(b"\x00" * (PASSWORD_STORAGE_SIZE - len(password_bytes)))  # Padding

    # b. Salt and Verifier
    try:
        binary_data.extend(bytes.fromhex(salt))
        binary_data.extend(bytes.fromhex(verifier))
    except ValueError:
        print('Error: Invalid hexadecimal characters found during conversion.')
        return None, None

    # Final check on size
    expected_size = 1 + PASSWORD_STORAGE_SIZE + SRP6A_SALT_LEN + SRP6A_VERIFIER_LEN
    if len(binary_data) != expected_size:
        print(f"Internal Error: Binary size mismatch. Expected {expected_size}, got {len(binary_data)}")
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


def find_partition_offset(partition_name):
    """
    Reads the partition-table.bin file and uses gen_esp32part.py to dump its contents
    for parsing the starting offset of the specified partition.
    """
    bin_path = build_dir / PARTITION_TABLE_REL_PATH
    tool_path = idf_path / PARTITION_TOOL_REL_PATH

    if not bin_path.exists():
        print(f'Error: Partition table binary not found at {bin_path}.')
        print('Please ensure your project has been built (idf.py build).')
        return None

    if not tool_path.exists():
        print(f"Error: Partition generation tool not found at {tool_path}.")
        return None

    print(f"Dumping partition table from binary using: {tool_path.name}")

    # CORRECTED COMMAND: Pass the binary file path directly without the '--dump' flag
    command = ["python", tool_path, bin_path]

    try:
        # Execute the dump command
        result = subprocess.run(command, capture_output=True, text=True, check=True, encoding="utf-8")
        csv_data = result.stdout
    except subprocess.CalledProcessError as err:
        print(f"Error executing gen_esp32part.py: {err.stderr}")
        return None

    # Use StringIO to treat the output string as a file object for line parsing
    csv_file = StringIO(csv_data)

    for line in csv_file:
        # Skip header/comment lines (which start with # or 'Parsing'/'Verifying')
        if (line.startswith("#")
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

            # Since the output is consistently '0x...', we can rely on that format.
            if re.match(r"0x[0-9a-fA-F]+$", offset_hex):
                print(f"Found partition '{partition_name}' at offset: {offset_hex}")
                return offset_hex

            # Catch case where size (5th element) might be mistaken for offset if parsing goes wrong
            # Your specific output showed the correct offset: '0x11000'
            print(f"Error parsing offset for '{partition_name}': {offset_hex}")
            return None

    print(f"Error: Partition '{partition_name}' not found in the dumped table data.")
    return None


def execute_flash_command(chip, port, offset_hex, binary_file):
    """flash the binary file"""
    print(f'Flashing {binary_file.name} to {chip} on {port} at offset {offset_hex}...')
    return esp_tool(["--chip", chip, "write_flash", offset_hex, str(binary_file)])


# --- MAIN EXECUTION ---

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Generate SRP6a credentials and flash to the custom partition.")

    parser.add_argument("--build_dir", required=True, help="Project build directory (e.g., C:/project/build).", )

    parser.add_argument('--idf_path', help="Path to IDF")

    parser.add_argument("--username", default=DEFAULT_USERNAME, help=f"Username (default = {DEFAULT_USERNAME})", )
    parser.add_argument("--password", help="Optional custom password (4 chars, A-Z/0-9) for testing.")

    parser.add_argument("--flash", action='store_true', help="Flash partition with security data.")
    parser.add_argument("--partition_name", default=DEFAULT_PARTITION_NAME,
                        help="Partition to flash with security data.")
    parser.add_argument("--port", help="The serial port of the device (e.g., COM3 or /dev/ttyUSB0).", )
    parser.add_argument("--chip", help="The target ESP chip type (e.g., esp32, esp32s3).", )

    args = parser.parse_args()

    try:
        if args.idf_path:
            idf_path = Path(args.idf_path)
        else:
            idf_path_env = os.getenv("IDF_PATH", None)
            if not idf_path_env:
                raise Exception("IDF_PATH environment variable not set.")
            idf_path = Path(idf_path_env)

        if args.flash and not (args.port and args.chip):
            raise Exception("Need --port and --chip and --partition_name if --flash specified")

        build_dir = Path(args.build_dir)

        # Password
        if args.password:
            password = validate_custom_password(args.password)
            print('--- USING CUSTOM PASSWORD FOR TESTING ---')
        else:
            if not args.port:
                raise Exception("--port required if passsord not specified")
            password = generate_mac_based_password()
            print('--- USING MAC ADDRESS PASSWORD (PRODUCTION MODE) ---')

        print(f'Provisioning Password: {password}')

        # Get salt, verifier output
        sec_output = get_sec_output(args.username, password)
        if not sec_output:
            raise Exception("Failed to get sec output")

        # If they don't want to flash it, just print it and quit
        # so they can copy it into source for development
        if not args.flash:
            print(sec_output)
            sys.exit(0)

        # Parse into binary
        salt_hex, verifier_hex = parse_sec_output(sec_output)

        if not (salt_hex and verifier_hex):
            raise Exception("Failed to parse sec output")

        # Create Unified Binary File
        output_dir = build_dir / "temp_prov"
        output_dir.mkdir(parents=True, exist_ok=True)

        binary_path = output_dir / TEMP_BINARY_FILENAME

        print(f'Generating unified binary file: {binary_path}...')
        final_binary_path = create_unified_binary(password, salt_hex, verifier_hex, binary_path)

        if not final_binary_path:
            raise Exception('Failed to create provisioning binary.')

        # Find partition offset for flashing
        offset = find_partition_offset(args.partition_name)

        if not offset:
            raise Exception('Could not determine flash offset from build directory.')

        # Flash partition
        if not execute_flash_command(args.chip, args.port, offset, final_binary_path):
            raise Exception('PROVISIONING FAILED!')

        print('Flashed successfully')
        print(f'User provisioning password: {password}')
        print(f'User provisioning username: {args.username}')

    except Exception as e:
        print(e, file=sys.stderr)
        sys.exit(1)
