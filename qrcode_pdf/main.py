from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Image
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import inch, mm
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.colors import black

# --- 1. Constants and Setup ---

# Define the custom page size (4x6 inches) in points (72 points per inch)
# ReportLab uses (width, height)
PAGE_WIDTH = 4 * inch
PAGE_HEIGHT = 6 * inch
FOUR_SIX = (PAGE_WIDTH, PAGE_HEIGHT)

# Input file names
QR_CODE_FILE = "../firmware/build/qr_code.png"  # Replace with your actual QR code file name
OUTPUT_FILE = "../firmware/build/qr_code_label.pdf"

# QR Code dimensions in pixels (328x328) - let's set it to 1.5 inches wide for the PDF
QR_SIZE_INCH = 25 * mm
QR_WIDTH_PX = 328
QR_HEIGHT_PX = 328

# Text content
TITLE_TEXT = "APP"
SUBTITLE_TEXT = "ESP BLE<br/>Provisioning"

# --- 2. Styling ---

styles = getSampleStyleSheet()

# Custom style for the centered Title
title_style = ParagraphStyle(
    name='TitleCentered',
    parent=styles['Heading1'],
    fontName='Helvetica-Bold',  # Change this to a custom font path if needed
    fontSize=16,
    alignment=TA_CENTER,
    spaceAfter=6,
    textColor=black
)

# Custom style for the centered Subtitle
subtitle_style = ParagraphStyle(
    name='SubtitleCentered',
    parent=styles['Normal'],
    fontName='Helvetica',
    fontSize=12,
    alignment=TA_CENTER,
    spaceAfter=12,  # Space after the subtitle, before the QR code
    textColor=black
)


# --- 3. Content Creation Function ---

def create_qr_pdf(qr_code_path, output_filename, title, subtitle):
    """Generates the PDF file with centered text and the QR code."""

    # Create the SimpleDocTemplate with the custom page size
    doc = SimpleDocTemplate(
        output_filename,
        pagesize=FOUR_SIX,
        leftMargin=0.25 * inch,  # Add small margins for safety
        rightMargin=0.25 * inch,
        topMargin=1 * mm,
        bottomMargin=0.5 * inch
    )

    story = []  # List to hold the flowable elements

    # 1. Add Centered Title Text
    p_title = Paragraph(title, title_style)
    story.append(p_title)

    # 2. Add Centered Subtitle Text
    p_subtitle = Paragraph(subtitle, subtitle_style)
    story.append(p_subtitle)

    # 3. Add Spacer (optional, adjust vertical spacing)
    # story.append(Spacer(1, 0.2 * inch))

    # 4. Add the QR Code Image
    # Load the image and set its display size in the PDF
    img = Image(
        qr_code_path,
        width=QR_SIZE_INCH,
        height=QR_SIZE_INCH,
        kind='proportional'
    )

    # ReportLab's platypus Image flowable centers the image automatically
    # if it's the only element on the line (which it is here).

    story.append(img)

    # Note: If you want to calculate the image position manually on a canvas
    # (not using SimpleDocTemplate), the centered x-coordinate would be:
    # x_centered = (PAGE_WIDTH - img.drawWidth) / 2

    # 5. Build the PDF document
    try:
        doc.build(story)
        print(f"✅ Successfully created PDF: {output_filename}")
    except FileNotFoundError:
        print(f"❌ Error: QR code file not found at {qr_code_path}")
    except Exception as e:
        print(f"❌ An error occurred: {e}")


# --- 4. Execution ---

if __name__ == "__main__":
    # Ensure your 'your_qr_code.png' file is in the same directory,
    # or provide the correct path.
    create_qr_pdf(QR_CODE_FILE, OUTPUT_FILE, TITLE_TEXT, SUBTITLE_TEXT)