#!/usr/bin/env python3
"""
Script to download rosbags from a Google Drive folder.
Downloads all files from a shared Google Drive folder to ~/.ros/pfms

Usage:
    python3 download_rosbags.py <google_drive_folder_url>
    
Example:
    python3 download_rosbags.py "https://drive.google.com/drive/folders/1a2b3c4d5e6f7g8h9i0j"
    
Requirements:
    pip install gdown
"""

import os
import sys
import argparse
from pathlib import Path


def extract_folder_id(url):
    """
    Extract Google Drive folder ID from URL.
    
    Supports formats:
    - https://drive.google.com/drive/folders/FOLDER_ID
    - https://drive.google.com/drive/folders/FOLDER_ID?usp=sharing
    """
    if '/folders/' in url:
        folder_id = url.split('/folders/')[1].split('?')[0]
        return folder_id
    else:
        # Assume it's already just the ID
        return url


def download_folder(folder_url, output_dir):
    """
    Download entire Google Drive folder to the specified directory.
    
    Args:
        folder_url: Google Drive folder URL or ID
        output_dir: Destination directory path
    """
    try:
        import gdown
    except ImportError:
        print("Error: 'gdown' module not found.")
        print("Please install it using: pip install gdown")
        sys.exit(1)
    
    # Extract folder ID
    folder_id = extract_folder_id(folder_url)
    print(f"Folder ID: {folder_id}")
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    print(f"Download directory: {output_dir}")
    
    # Construct the folder URL
    gd_url = f"https://drive.google.com/drive/folders/{folder_id}"
    
    print(f"\nDownloading from: {gd_url}")
    print("This may take a while depending on the folder size...\n")
    
    try:
        # Download the entire folder
        gdown.download_folder(url=gd_url, output=str(output_dir), quiet=False, use_cookies=False)
        print("\n✓ Download completed successfully!")
        print(f"Files saved to: {output_dir}")
        
        # List downloaded files
        files = list(Path(output_dir).rglob('*'))
        bag_files = [f for f in files if f.is_file() and (f.suffix == '.db3' or 'bag' in f.name.lower())]
        
        if bag_files:
            print(f"\nFound {len(bag_files)} rosbag file(s):")
            for bag_file in bag_files:
                size_mb = bag_file.stat().st_size / (1024 * 1024)
                print(f"  - {bag_file.name} ({size_mb:.2f} MB)")
        else:
            print("\nNote: No rosbag files (.db3) found in downloaded content.")
            all_files = [f for f in files if f.is_file()]
            if all_files:
                print(f"Downloaded {len(all_files)} file(s):")
                for f in all_files[:10]:  # Show first 10 files
                    print(f"  - {f.name}")
                if len(all_files) > 10:
                    print(f"  ... and {len(all_files) - 10} more")
        
    except Exception as e:
        print(f"\n✗ Error during download: {e}")
        print("\nTroubleshooting tips:")
        print("1. Make sure the folder is shared publicly or 'Anyone with the link can view'")
        print("2. Check that the URL is correct")
        print("3. Try upgrading gdown: pip install --upgrade gdown")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description='Download rosbags from Google Drive folder to ~/.ros/pfms',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s "https://drive.google.com/drive/folders/1a2b3c4d5e6f"
  %(prog)s "1a2b3c4d5e6f"
  %(prog)s "https://drive.google.com/drive/folders/1a2b3c4d5e6f" -o ~/custom/path
  %(prog)s    (uses default folder)
        """
    )
    
    parser.add_argument(
        'folder_url',
        nargs='?',
        default='https://drive.google.com/drive/folders/1v9DeQ9g2c26g9SSa7-eJxcp5kw-W8s8-?usp=sharing',
        help='Google Drive folder URL or folder ID (default: pfms rosbags folder)'
    )
    
    parser.add_argument(
        '-o', '--output',
        help='Output directory (default: ~/.ros/pfms)',
        default=None
    )
    
    args = parser.parse_args()
    
    # Determine output directory
    if args.output:
        output_dir = Path(args.output).expanduser()
    else:
        output_dir = Path.home() / '.ros' / 'pfms'
    
    print("=" * 60)
    print("  Google Drive Rosbag Downloader for PFMS")
    print("=" * 60)
    
    download_folder(args.folder_url, output_dir)


if __name__ == '__main__':
    main()
