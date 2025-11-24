from reportlab.lib.enums import TA_CENTER
from reportlab.platypus import SimpleDocTemplate, Paragraph, Image
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import inch, mm
from reportlab.lib.colors import black


def create_qr_pdf(qr_code_path, output_filename, title, subtitle, footer):
    PAGE_WIDTH = 4 * inch
    PAGE_HEIGHT = 6 * inch
    QR_SIZE = 25 * mm

    styles = getSampleStyleSheet()

    title_style = ParagraphStyle(
        name='Normal',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        alignment=TA_CENTER,
        fontSize=12,
        spaceAfter=3,
        textColor=black)

    subtitle_style = ParagraphStyle(
        name='SubtitleCentered',
        parent=styles['Normal'],
        fontName='Helvetica',
        alignment=TA_CENTER,
        fontSize=11,
        spaceAfter=3,
        textColor=black)

    pdf = [
        Paragraph(title, title_style),
        Paragraph(subtitle, subtitle_style),
        Image(str(qr_code_path), width=QR_SIZE, height=QR_SIZE, kind='proportional'),
        Paragraph(footer, subtitle_style),
    ]

    doc = SimpleDocTemplate(
        str(output_filename),
        pagesize=(PAGE_WIDTH, PAGE_HEIGHT),
        leftMargin=0.25 * inch,
        rightMargin=0.25 * inch,
        topMargin=1 * mm,
        bottomMargin=0.5 * inch
    )
    try:
        doc.build(pdf)
        print(f"Created PDF: {output_filename}")
    except FileNotFoundError:
        print(f"Error: QR code file not found at {qr_code_path}")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    QR_CODE_FILE = "../firmware/build/qr_code.png"  # Replace with your actual QR code file name
    OUTPUT_FILE = "../firmware/build/qr_code_label.pdf"
    TITLE_TEXT = "APP"
    SUBTITLE_TEXT = "ESP BLE<br/>Provisioning"
    FOOTER_TEXT = "PIN: 1234"
    create_qr_pdf(QR_CODE_FILE, OUTPUT_FILE, TITLE_TEXT, SUBTITLE_TEXT, FOOTER_TEXT)
