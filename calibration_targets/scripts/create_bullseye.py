from reportlab.lib.pagesizes import A3, A4
from reportlab.pdfgen import canvas

# Function to draw concentric circles
def draw_concentric_circles(pdf_canvas, center_x, center_y, num_circles, max_radius):
    for i in range(1, num_circles + 1):
        radius = max_radius * i / num_circles
        pdf_canvas.circle(center_x, center_y, radius, stroke=1, fill=0)

# Create an A3 PDF
def create_a3_pdf(filename):
    pdf_canvas = canvas.Canvas(filename, pagesize=A3)
    width, height = A3

    # Define parameters for A3
    center_x = width / 2
    center_y = height / 2
    max_radius = min(width, height) / 2 - 10  # Margin of 10 units
    num_circles = 50  # Adjust for A3

    draw_concentric_circles(pdf_canvas, center_x, center_y, num_circles, max_radius)
    pdf_canvas.save()

# Create an A4 PDF
def create_a4_pdf(filename):
    pdf_canvas = canvas.Canvas(filename, pagesize=A4)
    width, height = A4

    # Define parameters for A4
    center_x = width / 2
    center_y = height / 2
    max_radius = min(width, height) / 2 - 10  # Margin of 10 units
    num_circles = 40  # Adjust for A4

    draw_concentric_circles(pdf_canvas, center_x, center_y, num_circles, max_radius)
    pdf_canvas.save()

# Generate the PDFs
create_a3_pdf("concentric_circles_a3.pdf")
create_a4_pdf("concentric_circles_a4.pdf")