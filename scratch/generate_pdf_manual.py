import os
import sys
from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.lib.units import inch
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, KeepTogether, HRFlowable
)
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_CENTER, TA_LEFT, TA_RIGHT, TA_JUSTIFY

def build_pdf():
    pdf_path = "/Users/rafsanmallik/Desktop/IUB_DRONE/docs/OPERATOR_MANUAL_AND_SYSTEM_GUIDE.pdf"
    artifact_path = "/Users/rafsanmallik/.gemini/antigravity-ide/brain/19f0e92b-0302-4059-bfba-2f5eccf7ed6b/OPERATOR_MANUAL_AND_SYSTEM_GUIDE.pdf"
    
    os.makedirs(os.path.dirname(pdf_path), exist_ok=True)
    
    doc = SimpleDocTemplate(
        pdf_path,
        pagesize=letter,
        rightMargin=0.5*inch,
        leftMargin=0.5*inch,
        topMargin=0.5*inch,
        bottomMargin=0.5*inch
    )

    styles = getSampleStyleSheet()

    # Custom Palette
    PRIMARY_COLOR = colors.HexColor("#0f172a")    # Deep Slate/Navy
    ACCENT_CYAN    = colors.HexColor("#0284c7")    # Tactical Blue/Cyan
    ACCENT_RED     = colors.HexColor("#e11d48")    # Emergency Red
    TEXT_DARK      = colors.HexColor("#1e293b")    # Charcoal
    BG_LIGHT       = colors.HexColor("#f8fafc")    # Light slate bg
    BORDER_COLOR   = colors.HexColor("#cbd5e1")

    # Custom Styles
    title_style = ParagraphStyle(
        'DocTitle',
        parent=styles['Heading1'],
        fontName='Helvetica-Bold',
        fontSize=24,
        leading=28,
        textColor=PRIMARY_COLOR,
        alignment=TA_CENTER,
        spaceAfter=6
    )

    subtitle_style = ParagraphStyle(
        'DocSubTitle',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=12,
        leading=16,
        textColor=ACCENT_CYAN,
        alignment=TA_CENTER,
        spaceAfter=15
    )

    h1_style = ParagraphStyle(
        'H1',
        parent=styles['Heading2'],
        fontName='Helvetica-Bold',
        fontSize=15,
        leading=18,
        textColor=PRIMARY_COLOR,
        spaceBefore=14,
        spaceAfter=6,
        keepWithNext=True
    )

    h2_style = ParagraphStyle(
        'H2',
        parent=styles['Heading3'],
        fontName='Helvetica-Bold',
        fontSize=12,
        leading=15,
        textColor=ACCENT_CYAN,
        spaceBefore=10,
        spaceAfter=4,
        keepWithNext=True
    )

    body_style = ParagraphStyle(
        'Body',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=9.5,
        leading=13.5,
        textColor=TEXT_DARK,
        spaceAfter=6
    )

    bullet_style = ParagraphStyle(
        'Bullet',
        parent=body_style,
        leftIndent=15,
        firstLineIndent=-10,
        spaceAfter=4
    )

    code_style = ParagraphStyle(
        'CodeBlock',
        parent=styles['Normal'],
        fontName='Courier',
        fontSize=8.5,
        leading=11,
        textColor=colors.HexColor("#0f172a"),
        backColor=colors.HexColor("#f1f5f9"),
        borderColor=BORDER_COLOR,
        borderWidth=0.5,
        borderPadding=6,
        spaceBefore=4,
        spaceAfter=6,
        leftIndent=10,
        rightIndent=10
    )

    callout_style = ParagraphStyle(
        'Callout',
        parent=body_style,
        fontName='Helvetica-Oblique',
        fontSize=9,
        leading=13,
        textColor=colors.HexColor("#0369a1"),
        backColor=colors.HexColor("#e0f2fe"),
        borderColor=colors.HexColor("#38bdf8"),
        borderWidth=1,
        borderPadding=8,
        spaceBefore=6,
        spaceAfter=8
    )

    story = []

    # Title & Header Banner
    story.append(Paragraph("IUB DRONE COMMERCIAL AUTONOMY PLATFORM", title_style))
    story.append(Paragraph("OPERATOR MANUAL, SYSTEM INSTALLATION & NETWORK SETUP GUIDE<br/><font size=10 color='#64748b'>SUAS 2026 Tactical Ground Control Station & Autonomous Vision Suite</font>", subtitle_style))
    story.append(HRFlowable(width="100%", thickness=1.5, color=ACCENT_CYAN, spaceAfter=12))

    # Section 1
    story.append(Paragraph("1. System Architecture Overview", h1_style))
    story.append(Paragraph(
        "The <b>IUB Drone Commercial Autonomy Platform</b> is an aerospace-grade companion computer system designed for "
        "autonomous flight, vision perception, and Ground Control Station (GCS) telemetry operations. It connects onboard Jetson AI "
        "hardware directly to Matek ArduPilot flight controllers over high-speed serial MAVLink links.", body_style
    ))

    arch_data = [
        [Paragraph("<b>Component</b>", body_style), Paragraph("<b>Hardware Model / Protocol</b>", body_style), Paragraph("<b>Function & Specifications</b>", body_style)],
        [Paragraph("<b>Flight Controller</b>", body_style), Paragraph("Matek H743-WING V2 (ArduPilot)", body_style), Paragraph("400Hz MAVLink control over /dev/ttyTHS1 @ 921600 baud.", body_style)],
        [Paragraph("<b>Companion Computer</b>", body_style), Paragraph("NVIDIA Jetson Orin Nano / Nano", body_style), Paragraph("Ubuntu 22.04 LTS, MAXN 15W mode, PyTorch FP16 CUDA.", body_style)],
        [Paragraph("<b>Primary Camera</b>", body_style), Paragraph("SIYI A8 Mini 4K Gimbal Camera", body_style), Paragraph("HD RTSP stream (rtsp://192.168.144.25:8554/main.264) + USB fallback.", body_style)],
        [Paragraph("<b>Avoidance/VIO Cam</b>", body_style), Paragraph("Intel RealSense D455 3D", body_style), Paragraph("6-DOF VIO optical flow & 3D depth perception (/dev/video4).", body_style)],
        [Paragraph("<b>LiDAR Altimeter</b>", body_style), Paragraph("TFmini-S Micro LiDAR", body_style), Paragraph("Millimeter-precision altitude readout (/dev/ttyUSB0 @ 115200 baud).", body_style)],
        [Paragraph("<b>Ground Station</b>", body_style), Paragraph("Web GCS Asynchronous Engine", body_style), Paragraph("Zero-dependency dark-mode web workstation on port 8080.", body_style)],
    ]
    t_arch = Table(arch_data, colWidths=[1.5*inch, 2.2*inch, 3.8*inch])
    t_arch.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), BG_LIGHT),
        ('GRID', (0,0), (-1,-1), 0.5, BORDER_COLOR),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
    ]))
    story.append(t_arch)
    story.append(Spacer(1, 10))

    # Section 2
    story.append(Paragraph("2. Installation & Software Environment Setup", h1_style))
    story.append(Paragraph("Follow these step-by-step commands to install and optimize the Jetson companion computer environment:", body_style))
    
    install_code = """# 1. Clone the repository on Jetson companion computer
cd ~
git clone https://github.com/chishti055nsu/SUASS_DRONE.git IUB_DRONE
cd ~/IUB_DRONE

# 2. Grant serial port permissions for Matek FC and TFmini LiDAR
sudo usermod -a -G dialout,video $USER
sudo chmod 666 /dev/ttyTHS1 /dev/ttyUSB0 /dev/video* 2>/dev/null || true

# 3. Engage Max Hardware Performance (CPU & CUDA GPU Clock Lock)
bash scripts/maximize_jetson_performance.sh"""
    story.append(Paragraph(install_code.replace('\n', '<br/>').replace(' ', '&nbsp;'), code_style))

    # Section 3
    story.append(Paragraph("3. Ground Station to Drone Network Connection Setup", h1_style))
    story.append(Paragraph(
        "To establish high-speed wireless connectivity between the Ground Station computer (laptop/tablet) and the drone companion computer:", body_style
    ))

    story.append(Paragraph("• <b>IP Configuration</b>: Connect Ground Station computer and Jetson to the SIYI HM30 Datalink / WiFi router. Set Jetson Static IP to <b>192.168.144.100</b> and Ground Station IP to <b>192.168.144.150</b>.", bullet_style))
    story.append(Paragraph("• <b>Ping Verification</b>: Run <font face='Courier'>ping 192.168.144.100</font> from Ground Station to verify sub-millisecond network link.", bullet_style))
    story.append(Paragraph("• <b>RTSP Stream</b>: SIYI A8 Mini 4K stream broadcasts at <font face='Courier'>rtsp://192.168.144.25:8554/main.264</font>. If network RTSP is offline, system automatically falls back to USB video capture (/dev/video0).", bullet_style))

    story.append(Spacer(1, 8))

    # Section 4
    story.append(Paragraph("4. Web GCS Control Center Operation Manual", h1_style))
    story.append(Paragraph("Start the Web GCS server on the Jetson computer:", body_style))
    story.append(Paragraph("bash scripts/start_web_gcs.sh", code_style))
    story.append(Paragraph("Access the interface from your Ground Station browser at <b>http://192.168.144.100:8080</b>.", body_style))

    gcs_features = [
        [Paragraph("<b>Web GCS Tab</b>", body_style), Paragraph("<b>Primary Controls & Features</b>", body_style)],
        [Paragraph("<b>Top Telemetry Header</b>", body_style), Paragraph("Real-time 400Hz MAVLink status, 18 SAT RTK GPS, 25.2V 6S Power readout, and <b>clickable ARM / DISARM badge</b>.", body_style)],
        [Paragraph("<b>Tab 1: Tactical Dashboard</b>", body_style), Paragraph("2D ENU Radar with Click-to-Fly (GOTO), Horizon/Compass PFD, <b>🛑 DISARM MOTORS NOW</b>, <b>START MISSION</b>, <b>5.5KG HEAVY-LIFT</b>, and <b>Dynamic Hover Throttle Calibrator</b>.", body_style)],
        [Paragraph("<b>Tab 2: SIYI A8 Mini Feed</b>", body_style), Paragraph("Live HD RTSP feed with FP16 YOLOv8 object detection, Target-Payload Matching table (Mannequin -> Water Bottle), and Gemma AI Scene Reasoning.", body_style)],
        [Paragraph("<b>Tab 3: RealSense D455</b>", body_style), Paragraph("3D VIO optical flow tracking, 3-Sector Obstacle Avoidance clearance (Left/Center/Right), and TFmini-S LiDAR altitude indicator.", body_style)],
        [Paragraph("<b>Tab 4: Virtual Remote</b>", body_style), Paragraph("Touch/Mouse Teleop Nudge Controls and Precision Throttle Override slider.", body_style)]
    ]
    t_gcs = Table(gcs_features, colWidths=[2.2*inch, 5.3*inch])
    t_gcs.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), BG_LIGHT),
        ('GRID', (0,0), (-1,-1), 0.5, BORDER_COLOR),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
    ]))
    story.append(t_gcs)
    story.append(Spacer(1, 10))

    # Section 5
    story.append(Paragraph("5. Step-by-Step Flight Operations & Emergency Procedures", h1_style))
    story.append(Paragraph("<b>Pre-Flight Checklist:</b>", h2_style))
    story.append(Paragraph("1. Check 6S LiPo battery voltage (&ge; 24.0V).", bullet_style))
    story.append(Paragraph("2. Verify MAVLink 400Hz connection and 3D RTK GPS Fix (&ge; 14 Satellites).", bullet_style))
    story.append(Paragraph("3. Verify TFmini-S LiDAR altitude readout and SIYI A8 Mini camera feed.", bullet_style))

    story.append(Paragraph("<b>Emergency Motor Cut-Off Procedure:</b>", h2_style))
    callout_text = (
        "<b>EMERGENCY DISARM PROCEDURE:</b><br/>"
        "If a flight anomaly occurs, click <b>🛑 DISARM MOTORS NOW</b> on the Web GCS control panel or click the top header status badge <b>ARMED - LIVE</b>.<br/>"
        "<b>Triple-Redundant Safety Action Triggered:</b><br/>"
        "1. Direct MAVLink force-disarm command (param1=0, param2=21196 force magic) sent over serial.<br/>"
        "2. Throttle PWM locked to 1000 across all channels.<br/>"
        "3. ROS 2 MAVROS arming service call (/mavros/cmd/arming value: false) dispatched.<br/>"
        "<b>Motors cut power instantly!</b>"
    )
    story.append(Paragraph(callout_text, callout_style))

    # Section 6
    story.append(Paragraph("6. Quick Troubleshooting Guide", h1_style))
    trouble_data = [
        [Paragraph("<b>Symptom</b>", body_style), Paragraph("<b>Root Cause</b>", body_style), Paragraph("<b>Resolution</b>", body_style)],
        [Paragraph("Web GCS page not loading", body_style), Paragraph("Server not started or port blocked.", body_style), Paragraph("Run 'bash scripts/start_web_gcs.sh' on Jetson. Check IP.", body_style)],
        [Paragraph("SIYI A8 Mini stream black", body_style), Paragraph("RTSP IP mismatch or cable unplugged.", body_style), Paragraph("System auto-falls back to USB video (/dev/video0). Check Ethernet IP.", body_style)],
        [Paragraph("Tab switch lag", body_style), Paragraph("Dual streams downloading simultaneously.", body_style), Paragraph("Updated software automatically pauses hidden stream on tab switch.", body_style)],
        [Paragraph("Motors not disarming", body_style), Paragraph("Serial connection lost.", body_style), Paragraph("Click '🛑 DISARM MOTORS NOW' or header badge for triple-redundant disarm.", body_style)]
    ]
    t_trouble = Table(trouble_data, colWidths=[1.8*inch, 2.2*inch, 3.5*inch])
    t_trouble.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), BG_LIGHT),
        ('GRID', (0,0), (-1,-1), 0.5, BORDER_COLOR),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
    ]))
    story.append(t_trouble)

    doc.build(story)
    
    # Also copy to artifact path
    with open(pdf_path, 'rb') as f_in:
        pdf_bytes = f_in.read()
    with open(artifact_path, 'wb') as f_out:
        f_out.write(pdf_bytes)

    print(f"Successfully generated PDF: {pdf_path}")
    print(f"Copied to artifacts: {artifact_path}")

if __name__ == "__main__":
    build_pdf()
