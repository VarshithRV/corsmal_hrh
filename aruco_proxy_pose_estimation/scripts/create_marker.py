import cv2
import numpy as np
from fpdf import FPDF

# Define the dictionary and marker size
# aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
marker_size = 330  # Marker size in pixels

# Create markers with different IDs
marker_ids = [1,2,3,21]
markers = []

for marker_id in marker_ids:
    tag = np.zeros((300, 300, 1), dtype="uint8")
    marker_img = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250), marker_id, 300)
    markers.append(marker_img)

# Create a PDF document
pdf = FPDF()
pdf.add_page()

# Add markers to the PDF
for i, marker_img in enumerate(markers):
    # Save marker image to a temporary file
    filename = f"marker_{marker_ids[i]}.png"
    cv2.imwrite(filename, marker_img)
    
    # Add marker image to the PDF
    pdf.image(filename, x=10, y=5 + i * (marker_size / 5 + 5), w=marker_size / 5, h=marker_size / 5)

# Save the PDF
pdf.output("aruco_markers.pdf")
print("PDF with ArUco markers created successfully.")