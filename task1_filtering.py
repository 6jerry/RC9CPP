import cv2
import numpy as np
import os
from utils import plot_images

def process_task1(image_path, output_dir):
    # Read the noisy image with Chinese path support
    img = cv2.imdecode(np.fromfile(image_path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(f"Image not found at {image_path}")
    
    # Store results for different filters
    kernel_sizes = [3, 5, 7, 9, 11]
    
    # 1. Mean Filter (均值滤波)
    mean_results = [img]
    mean_titles = ['Original']
    for k in kernel_sizes:
        result = cv2.blur(img, (k, k))
        mean_results.append(result)
        mean_titles.append(f"Mean {k}x{k}")
    
    plot_images(mean_results, mean_titles, 2, 3, "Mean Filter Comparison", "mean_filter.png", output_dir)
    
    # 2. Median Filter (中值滤波)
    median_results = [img]
    median_titles = ['Original']
    for k in kernel_sizes:
        result = cv2.medianBlur(img, k)
        median_results.append(result)
        median_titles.append(f"Median {k}x{k}")
        
    plot_images(median_results, median_titles, 2, 3, "Median Filter Comparison", "median_filter.png", output_dir)
    
    # 3. Gaussian Filter (高斯滤波)
    gauss_results = []
    gauss_titles = []
    gauss_kernel_sizes = [3, 5, 7]
    sigmas = [0.5, 1.0, 1.5, 2.0]
    
    for k in gauss_kernel_sizes:
        for sigma in sigmas:
            result = cv2.GaussianBlur(img, (k, k), sigma)
            gauss_results.append(result)
            gauss_titles.append(f"k={k}, s={sigma}")
            
    plot_images([img] + gauss_results, ['Original'] + gauss_titles, 3, 5, "Gaussian Filter Comparison", "gaussian_filter.png", output_dir)
    
    # 4. Compare all three
    best_mean = cv2.blur(img, (5, 5))
    best_median = cv2.medianBlur(img, 5)
    best_gauss = cv2.GaussianBlur(img, (5, 5), 1.0)
    
    comp_images = [img, best_mean, best_gauss, best_median]
    comp_titles = ['Original (Noisy)', 'Mean (5x5)', 'Gaussian (5x5, s=1.0)', 'Median (5x5)']
    
    plot_images(comp_images, comp_titles, 2, 2, "Comparison of Three Filters (5x5)", "filter_comparison.png", output_dir)

if __name__ == "__main__":
    img_path = r"d:\Destop\图像处理大作业\images\image1.png"
    out_dir = r"d:\Destop\图像处理大作业\results\task1"
    process_task1(img_path, out_dir)
    print("Task 1 completed successfully.")
