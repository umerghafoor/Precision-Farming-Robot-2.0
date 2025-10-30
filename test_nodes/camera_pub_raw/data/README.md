# Image Data Folder

Place your test images in this folder. The image publisher will automatically load and publish all images found here.

## Supported Formats

- JPG/JPEG
- PNG
- BMP
- TIFF/TIF

## Usage

1. Copy your images to this folder:
   ```bash
   cp /path/to/your/images/*.jpg .
   ```

2. The images will be published in alphabetical order

3. You can verify what images are loaded by checking the node's startup logs

## Current Images

Images currently in this folder:
- PXL_20251027_082153575.RAW-01.COVER~3.jpg

## Tips

- Use descriptive filenames (e.g., `farm_field_01.jpg`, `crop_close_up_02.jpg`)
- Images will be published with their original resolution unless you use the `resize_width` and `resize_height` parameters
- Larger images will consume more bandwidth on the ROS2 network
- For testing, consider using smaller resolution images (e.g., 640x480 or 1280x720)

## Adding More Images

You can download sample farming/agricultural images or use your own camera images. Some ideas:

- Crop field images
- Plant close-ups
- Agricultural machinery
- Soil samples
- Pest/disease examples
- Weather conditions

Just make sure they're in one of the supported formats!
