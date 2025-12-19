import os
import torch
import cv2
import numpy as np
import sys
from torchvision.transforms import Normalize

from sam2.build_sam import build_sam2
from keypoint_detector.gamma_correction import process_dimmed, process_bright


import os 
import torch
import cv2
import numpy as np


## Forward Network


class ContourFinder():
    """
    Barebones PiDiNet edge detection for real-time camera streams.
    Optimized for OpenCV input and single-frame inference.
    """

    def __init__(self, checkpoint_path='pidinet/trained_models/table7_pidinet.pth',):
        """
        Initialize the edge detector
        
        Args:
            checkpoint_path: Path to pretrained model weights
        """
        enhance_pretrain = 'keypoint_detector/best_Epoch_lol_v1.pth'
        exposure_pretrain = 'keypoint_detector/best_Epoch_exposure.pth'

        ## Load Pre-train Weights
        
        
    def auto_canny(self, image, sigma=0.33):
        """
        Automatically determine Canny thresholds based on image median.
        """
        # Compute median of pixel intensities
        median = np.median(image)
        
        # Calculate lower and upper thresholds
        lower = int(max(0, (1.0 - sigma) * median))
        upper = int(min(255, (1.0 + sigma) * median))
        
        # Apply Canny
        edges = cv2.Canny(image, lower, upper)
        
        return edges

    def enhance_realsense_image(self, img):
        """
        Enhance RealSense color frame before feeding to PiDiNet
        """
        
        img = cv2.cvtColor(np.array(img), cv2.COLOR_BGR2RGB)
        img = (np.asarray(img)/ 255.0)
        if img.shape[2] == 4:
            img = img[:,:,:3]
        input_img = torch.from_numpy(img).float().cuda()
        input_img = input_img.permute(2,0,1).unsqueeze(0)
        # if config.normalize:    # False
        
        _, _ ,enhanced_img = self.model(input_img)
        # 1. Move to CPU and detach from graph
        out = enhanced_img.squeeze(0).detach().cpu()   # shape: [3, H, W]

        # 2. Convert to numpy and permute to HWC
        out = out.permute(1, 2, 0).numpy()             # shape: [H, W, 3]

        # 3. Clamp values to valid range
        out = np.clip(out, 0, 1)

        # 4. Convert RGB -> BGR for OpenCV
        out_bgr = out[..., ::-1]

        # 5. Convert to uint8
        out_bgr = (out_bgr * 255).astype(np.uint8)
        return out_bgr

    # def process_img(self, img, idx, offset):
    #     """
    #     Process a single OpenCV image (BGR format)
    #     Args:
    #         img: OpenCV image (numpy array, BGR format)
    #     Returns:
    #         processed: binary edge map ready for circle detection
    #     """
    #     print("IMG shape", img.shape)
    #     # lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    #     # L, A, B = cv2.split(lab)

    #     # clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(4,4))
    #     # # L_enh = clahe.apply(L)
    #     # img = clahe.apply(img)
    #     # enh_lab = cv2.merge([L_enh, A, B])
    #     # img = cv2.cvtColor(img, cv2.COLOR_LAB2BGR)

    #     def adjust_gamma_and_suppress_bgr(image, gamma=0.4, bright_thresh=230):
    #         """
    #         Apply gamma correction and suppress overly bright (white) pixels.
    #         Input and output are both in BGR format.

    #         Args:
    #             image (np.ndarray): Input BGR image.
    #             gamma (float): Gamma correction factor (<1 brightens, >1 darkens).
    #             bright_thresh (int): Brightness threshold (0–255) above which pixels are set to black.

    #         Returns:
    #             np.ndarray: Processed image (BGR).
    #         """
    #         # --- Step 1: Gamma correction ---
    #         print("Input image shape:", image.shape)
    #         invGamma = 1.0 / gamma
    #         table = np.array([(i / 255.0) ** invGamma * 255 for i in np.arange(256)]).astype("uint8")
    #         gamma_corrected = cv2.LUT(image, table)
    #         print("error-1")
    #         # --- Step 2: Bright pixel suppression ---
    #         # Convert to grayscale to find bright spots
    #         gray = cv2.cvtColor(gamma_corrected, cv2.COLOR_BGR2GRAY)
    #         print("error123")
    #         print(gamma_corrected.shape)
    #         mask = gray > bright_thresh
    #         print("error0")
    #         # Optionally expand the mask slightly (to remove halos)
    #         mask = cv2.dilate(mask.astype(np.uint8), np.ones((3, 3), np.uint8), iterations=1)
    #         print("error1")
    #         # Set bright pixels to black in BGR
    #         result_bgr = gamma_corrected.copy()
    #         result_bgr[mask > 0] = (0, 0, 0)

    #         # ✅ Ensure output is still BGR
    #         return result_bgr


        
    #     img = adjust_gamma_and_suppress_bgr(img, gamma=4, bright_thresh=200)
    #     cv2.imshow("gamma", img)
    #     cv2.waitKey(1)
    #     image_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    #     if image_rgb.dtype != np.uint8:
    #         image_rgb = np.clip(image_rgb, 0, 255).astype(np.uint8)
            
    #     self.predictor.set_image(image_rgb)
    #     masks, _, _ = self.predictor.predict(box=offset, multimask_output=False)
    #     mask = masks[0] if masks.ndim == 3 else masks  # pick first mask if batched

    #     # Convert boolean or float mask → uint8
    #     mask_vis = (mask * 255).astype(np.uint8)

    #     # # If single channel, convert to 3-channel for color display
    #     # if len(mask_vis.shape) == 2:
    #     #     mask_vis = cv2.cvtColor(mask_vis, cv2.COLOR_GRAY2BGR)
    #     print("Finished proces_img")
    #     return mask_vis
    def process_with_pidi(self, img, idx):
        if img.shape == 2:
            img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        else:
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
        img_rgb = img_rgb.astype(np.float32) / 255.0


        # print("Before image torch")
        # === [2] Tensor conversion and ImageNet normalization ===
        img_tensor = torch.from_numpy(img_rgb).permute(2, 0, 1)
        mean = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
        std = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)
        img_tensor = (img_tensor - mean) / std
        img_tensor = img_tensor.unsqueeze(0).to(self.device)
        # print("After image nupmy")

        # # === [3] PiDiNet inference ===
        with torch.no_grad():
            results = self.model(img_tensor)
            edge_map = results[-1].squeeze().cpu().numpy()
            # edge_map = cv2.normalize(edge_map, None, 0, 255, cv2.NORM_MINMAX)
            # edge_map = (edge_map).astype(np.uint8)
            # edge_map = cv2.Canny(edge_map, 50, 150)
            cv2.imshow(f"pidinet_raw_{idx}", edge_map)
            cv2.waitKey(1)
        
    
    def fold_highlights(self, img_bgr, channel='gray', keep_above_mode=5, strength=1.0):
        """
        img_bgr         : input BGR image (uint8)
        channel         : 'gray'  -> work on grayscale
                        'value' -> work on V channel of HSV
        keep_above_mode : range above the mode that is left almost unchanged
        strength        : how strongly to push highlights down
                        (1.0 = strong, 0.5 = milder, etc.)
        """
        work = img_bgr
        # --- 1. pick working channel (grayscale or HSV Value) ---
        # if channel == 'gray':
        #     work = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        # elif channel == 'value':
        #     hsv  = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        #     work = hsv[:, :, 2]
        # else:
        #     raise ValueError("channel must be 'gray' or 'value'")

        # --- 2. find most common brightness (plastic body) ---
        hist = cv2.calcHist([work], [0], None, [256], [0, 256])
        mode_val = int(np.argmax(hist))  # 0..255

        # --- 3. build LUT that keeps around the mode, folds bright values down ---
        lut = np.zeros((256,), dtype=np.uint8)

        border = min(255, mode_val + keep_above_mode)

        for i in range(256):
            if i <= border:
                # keep dark & mid tones unchanged
                lut[i] = i
            else:
                # fold highlights back toward black, mirroring around mode
                d = i - border               # distance above border
                val = mode_val - int(strength * d)
                if val < 0:
                    val = 0
                lut[i] = val

        # --- 4. apply LUT ---
        out = cv2.LUT(work, lut)

        # --- 5. put back into image ---
        # if channel == 'gray':
        #     out = cv2.cvtColor(work_folded, cv2.COLOR_GRAY2BGR)
        # else:  # 'value'
        #     hsv[:, :, 2] = work_folded
        #     out = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        return out, lut, mode_val 
        
    def process_img(self, img, visualize=False):
        """
        Process a single OpenCV image (BGR format)
        Args:
            img: OpenCV image (numpy array, BGR format)
        Returns:
            processed: binary edge map ready for circle detection
        """
        # === [1] RGB conversion and normalization ===
        # img = cv2.bilateralFilter(img, 5, 20, 20)
        # clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(4,4))
        # # L_enh = clahe.apply(L)
        # img = clahe.apply(img)
        

        def adjust_gamma_and_suppress_bgr(img):
            """
            Apply gamma correction and suppress overly bright (white) pixels.
            Input and output are both in BGR format.

            Args:
                image (np.ndarray): Input BGR image.
                gamma (float): Gamma correction factor (<1 brightens, >1 darkens).
                bright_thresh (int): Brightness threshold (0–255) above which pixels are set to black.

            Returns:
                np.ndarray: Processed image (BGR).
            """
            # --- Step 1: Gamma correction ---
                
            YCrCb = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
            Y = YCrCb[:,:,0]
            # Determine whether image is bright or dimmed
            threshold = 1
            exp_in = 25 # Expected global average intensity 
            M,N = img.shape[:2]
            mean_in = np.sum(Y/(M*N)) 
            t = (mean_in - exp_in)/ exp_in
            
            # Process image for gamma correction
            img_output = None
            if t < threshold: # Dimmed Image
                result = process_dimmed(Y)
                YCrCb[:,:,0] = result
                img_output = cv2.cvtColor(YCrCb,cv2.COLOR_YCrCb2BGR)
            elif t > threshold:
                result = process_bright(Y)
                YCrCb[:,:,0] = result
                img_output = cv2.cvtColor(YCrCb,cv2.COLOR_YCrCb2BGR)
            else:
                img_output = img
                
            return img_output
        
        def SSR(img, sigma=20):
            # convert to float
            img = img.astype(np.float32) + 1.0  

            # Gaussian blur = illumination estimate
            blur = cv2.GaussianBlur(img, (0,0), sigma)

            # Retinex = log(image) - log(illumination)
            retinex = np.log(img) - np.log(blur + 1)

            # scale to 0–255
            retinex = (retinex - retinex.min()) / (retinex.max() - retinex.min())
            retinex = (retinex * 255).astype(np.uint8)

            return retinex
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        # # L_enh = clahe.apply(L)
        # img = clahe.apply(img)
        
        img = img * 0.6 - 5    # strong darkening
        img = np.clip(img, 0, 255).astype(np.uint8)
                
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # # split channels
        h, s, v = cv2.split(hsv)
        current_mean = np.mean(v)
        target_mean = 30
        scale = target_mean / (current_mean + 1e-6)
        v = np.clip(v * scale , 0, 255).astype(np.uint8)
        img = cv2.merge([h, s, v])
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        
        
        # lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        # l, a, b = cv2.split(lab)
        # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(11,11))
        # l2 = clahe.apply(l)
        # lab2 = cv2.merge([l2, a, b])
        # img = cv2.cvtColor(lab2, cv2.COLOR_LAB2BGR)
        # mask = mask_by_brightness(img, 125)
        # img = cv2.bitwise_and(img, img, mask=mask)  # Apply mask

        img = adjust_gamma_and_suppress_bgr(img)
        cv2.imshow("before gamma 2.5", img)
        
        # pixels = img.reshape(-1, 3).astype(np.float32)

        # # K-means to find dominant color
        # K = 1  # 1 cluster = most common color
        # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

        # _, _, centers = cv2.kmeans(pixels, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        # dominant_color = centers[0]  # R,G,B center of cluster
        
        # diff = img - dominant_color
        # d = np.sqrt(np.sum(diff * diff, axis=2))  # distance map
        # d_norm = d / np.sqrt(3 * 255 * 255)       # normalize 0–1


        # sigma = 0.6  # adjust to taste
        # w = np.exp(-(d_norm ** 2) / (sigma ** 2))
        # w = w[..., None]  # broadcast to 3 channels

        # img = img * w
        # img = np.clip(img, 0, 255).astype(np.uint8)


        
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # img = cv2.bilateralFilter(img, 5, 30, 30)    

        num_it = 1
        for i in range(num_it):
        #     img, _, _ = self.fold_highlights(img, "gray", 1, 0.05)
            img = cv2.bilateralFilter(img, 5, 30, 30)    
            
        #     # clahe = cv2.createCLAHE(clipLimit=2, tileGridSize=(11, 11))
        #     # img = clahe.apply(img)
            img = cv2.GaussianBlur(img, (5,5), 1)
            

        #     # 2) Slight sharpen (unsharp mask)
            blur = cv2.GaussianBlur(img, (0, 0), sigmaX=1.0)
            img = cv2.addWeighted(img, 1.5, blur, -1, 0)
            
            # if i < num_it - 1:
            #     img = SSR(img, 1000)
            img = cv2.normalize(
                img, None,
                alpha=0, beta=255,
                norm_type=cv2.NORM_MINMAX
            ).astype('uint8')
        
            
        # clahe = cv2.createCLAHE(clipLimit=2, tileGridSize=(9, 9))
        # img = clahe.apply(img)
            
            
            
            
            
        # # img = improve_dog_edges(img)
        # median = np.median(img)
        
        # img[img < (median)] = 0

        # # # Strength of darkening (0 = none, 1 = maximum)
        # # k = 1

        # # # Darken based on distance from median
        # # img = img * (1 - k * d)

        # # img = np.clip(img, 0, 255).astype(np.uint8)
        # blur = cv2.GaussianBlur(img, (11, 11), 2)

        cv2.imshow("before gamma", img)
        # cv2.waitKey(1)

        edge = cv2.adaptiveThreshold(
            img,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            21,  # block size (must be odd)
            3  # C value
        )
        # edge = ((img > 0)*255).astype(np.uint8)
        
        edge = cv2.bitwise_not(edge)
        # # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        # # edge = cv2.morphologyEx(edge, cv2.MORPH_OPEN, kernel, iterations=1)
        # # Smooth a bit to merge thin edges and remove salt-and-pepper noise
        # # edge = cv2.bilateralFilter(edge, d=9, sigmaColor=75, sigmaSpace=75)
        # img = cv2.medianBlur(img, 5)


        # # Optional: erode small specks, then dilate to close gaps
        # # kernel = np.ones((4,4), np.uint8)
        # # edge = cv2.morphologyEx(edge, cv2.MORPH_OPEN, kernel)
        
        # # cv2.dilate(edge, np.ones((2,2), np.uint8), iterations=1)

        # # edge = cv2.medianBlur(edge, 5)
        # # kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        # # edge = cv2.morphologyEx(edge, cv2.MORPH_OPEN, kernel_small, iterations=1)

        # # # Connect broken edges with closing
        
        # # kernel_medium = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        # # edge = cv2.morphologyEx(edge, cv2.MORPH_CLOSE, kernel_medium, iterations=1)
        cv2.imshow("adaptive_binary2", edge)

        edge = cv2.Canny(edge, 50, 150)
        # # # kernel = np.ones((2,2), np.uint8)   # choose kernel size
        # # # edge = cv2.morphologyEx(edge, cv2.MORPH_CLOSE, kernel)
        cv2.imshow("adaptive_binary", edge)

        return edge
    
# Example usage for camera stream
if __name__ == '__main__':
    # Initialize detector
    detector = ContourFinder()
    
    # Open camera
    cap = cv2.VideoCapture(0)
    
    print("Camera stream started. Press 'q' to quit")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Get edge map (values 0-1, float)
        edge_map = detector.process_img(frame)
        
        # Option 1: Use as probability map (0-1 range)
        # You can threshold later: edges_binary = edge_map > 0.5
        
        # Option 2: Convert to uint8 for display
        edge_display = (edge_map * 255).astype(np.uint8)
        
        # Display
        cv2.imshow('Original', frame)
        cv2.imshow('Edges', edge_display)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()