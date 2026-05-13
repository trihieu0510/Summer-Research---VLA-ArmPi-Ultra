import cv2

def convert_to_grayscale(input_path: str, output_path: str) -> None:
    # Read the image in color
    image = cv2.imread(input_path)
    if image is None:
        raise FileNotFoundError(f"Could not read image at {input_path}")

    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Save the grayscale image
    success = cv2.imwrite(output_path, gray)
    if not success:
        raise RuntimeError(f"Could not write image to {output_path}")

if __name__ == "__main__":
    input_file = "miule.jpg"
    output_file = "output_gray.jpg"
    convert_to_grayscale(input_file, output_file)
    print(f"Saved grayscale image to {output_file}")