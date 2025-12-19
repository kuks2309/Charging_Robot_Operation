import cv2
import numpy as np
from cv2 import aruco

def create_aruco_grid(grid_width=4, grid_height=3, marker_size=100, 
                      marker_separation=20, dictionary_id=aruco.DICT_6X6_250,
                      dpi=300, save_path="aruco_grid.png"):
    """
    Create and save an ArUco marker grid for printing
    
    Args:
        grid_width: Number of markers horizontally
        grid_height: Number of markers vertically  
        marker_size: Size of each marker in pixels
        marker_separation: Separation between markers in pixels
        dictionary_id: ArUco dictionary to use
        dpi: DPI for printing (300 recommended for high quality)
        save_path: Where to save the generated image
    """
    
    # Create ArUco dictionary
    dictionary = aruco.getPredefinedDictionary(dictionary_id)
    
    # Create the grid board
    board = aruco.GridBoard((grid_width, grid_height), 
                           marker_size, marker_separation, dictionary)
    
    # Calculate image size needed
    img_width = grid_width * marker_size + (grid_width - 1) * marker_separation
    img_height = grid_height * marker_size + (grid_height - 1) * marker_separation
    
    # Add margins (10% of marker size)
    margin = int(marker_size * 0.1)
    img_width += 2 * margin
    img_height += 2 * margin
    
    # Generate the board image
    img = board.generateImage((img_width, img_height), marginSize=margin)
    
    # Save the image
    cv2.imwrite(save_path, img)
    print(f"ArUco grid saved to {save_path}")
    print(f"Grid: {grid_width}x{grid_height} markers")
    print(f"Image size: {img_width}x{img_height} pixels")
    print(f"Print at {dpi} DPI for best results")
    
    # Calculate physical dimensions at given DPI
    width_inches = img_width / dpi
    height_inches = img_height / dpi
    print(f"Physical size at {dpi} DPI: {width_inches:.2f}\" x {height_inches:.2f}\"")
    
    return img, board

def create_charuco_grid(grid_width=7, grid_height=5, square_size=40,
                        marker_size=30, dictionary_id=aruco.DICT_6X6_250,
                        dpi=300, save_path="charuco_grid.png"):
    """
    Create a ChArUco board (combines ArUco + chessboard for better detection)
    
    Args:
        grid_width: Number of squares horizontally
        grid_height: Number of squares vertically
        square_size: Size of each chessboard square in pixels
        marker_size: Size of ArUco markers in pixels
        dictionary_id: ArUco dictionary to use
        dpi: DPI for printing
        save_path: Where to save the generated image
    """
    
    # Create ArUco dictionary
    dictionary = aruco.getPredefinedDictionary(dictionary_id)
    
    # Create ChArUco board
    board = aruco.CharucoBoard((grid_width, grid_height), 
                              square_size, marker_size, dictionary)
    
    # Calculate image size
    img_width = grid_width * square_size
    img_height = grid_height * square_size
    
    # Add margins
    margin = int(square_size * 0.1)
    img_width += 2 * margin
    img_height += 2 * margin
    
    # Generate the board image
    img = board.generateImage((img_width, img_height), marginSize=margin)
    
    # Save the image
    cv2.imwrite(save_path, img)
    print(f"ChArUco board saved to {save_path}")
    print(f"Grid: {grid_width}x{grid_height} squares")
    print(f"Image size: {img_width}x{img_height} pixels")
    
    # Calculate physical dimensions
    width_inches = img_width / dpi
    height_inches = img_height / dpi
    print(f"Physical size at {dpi} DPI: {width_inches:.2f}\" x {height_inches:.2f}\"")
    
    return img, board

def detect_grid(image_path, board, camera_matrix=None, dist_coeffs=None):
    """
    Example detection function for the generated grid
    """
    # Read image
    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Create detector with default parameters
    detector = aruco.ArucoDetector(board.getDictionary())
    
    # Detect markers
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None:
        print(f"Detected {len(ids)} markers")
        
        # If camera is calibrated, estimate pose
        if camera_matrix is not None and dist_coeffs is not None:
            # Estimate pose of the board
            retval, rvec, tvec = aruco.estimatePoseBoard(
                corners, ids, board, camera_matrix, dist_coeffs, None, None)
            
            if retval > 0:
                print(f"Board pose estimated using {retval} markers")
                # Draw axis on the board
                img = aruco.drawAxis(img, camera_matrix, dist_coeffs, 
                                   rvec, tvec, 100)
        
        # Draw detected markers
        img = aruco.drawDetectedMarkers(img, corners, ids)
        
        return img, len(ids)
    else:
        print("No markers detected")
        return img, 0

if __name__ == "__main__":
    # Example 1: Create a simple ArUco grid
    print("Creating ArUco grid...")
    img1, board1 = create_aruco_grid(
        grid_width=4, 
        grid_height=3, 
        marker_size=80,
        marker_separation=20,
        dpi=300,
        save_path="aruco_4x3_grid.png"
    )
    
    print("\n" + "="*50 + "\n")
    
    # Example 2: Create a ChArUco board (more reliable)
    print("Creating ChArUco board...")
    img2, board2 = create_charuco_grid(
        grid_width=8,
        grid_height=6, 
        square_size=352,
        marker_size=247,
        dpi=300,
        save_path="charuco_8x6_board.png"
    )
    
    print("\n" + "="*50 + "\n")
    print("Printing tips:")
    print("1. Print on matte paper to reduce glare")
    print("2. Use the highest quality print setting")
    print("3. Ensure 100% scale (no 'fit to page')")
    print("4. Mount on rigid surface for best results")
    print("5. ChArUco boards are generally more reliable than pure ArUco grids")
    print("size of 66 is 0.00559 m nad of 70 is 0.00796 m")