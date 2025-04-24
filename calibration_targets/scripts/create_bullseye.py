from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm

def draw_concentric_circles_pdf(filename="concentric_circles.pdf"):
    # Define page size
    width, height = A4  # A4 size in points

    # Set center of the page
    center_x = width / 2
    center_y = height / 2

    # Determine maximum radius in mm (smallest distance from center to any edge)
    max_radius_mm = min(center_x, center_y) / mm

    # Create a canvas
    c = canvas.Canvas(filename, pagesize=A4)

    # Draw circles with radius from 1mm to max_radius_mm
    for r_mm in range(1, int(max_radius_mm) + 1):
        r_pt = r_mm * mm
        # Set line width: thicker every 10 mm (i.e., every 1 cm)
        if r_mm % 10 == 0:
            c.setLineWidth(1.2)  # Thicker line
        else:
            c.setLineWidth(0.3)  # Thin line

        c.circle(center_x, center_y, r_pt)

    # Save the canvas
    c.save()
    print(f"PDF saved as: {filename}")

if __name__ == "__main__":
    draw_concentric_circles_pdf()
