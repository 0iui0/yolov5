import json
import glob
from pathlib import Path
import pandas as pd

input_path = "../datasets/bot-seg/labels/train2023/*.json"
output_path = "../datasets/bot-seg/labels/train2023/"

def convert_json_to_yolo(json_file):
    image_width = 0
    image_height = 0
    with open(json_file, 'r') as f:
        data = json.load(f)
        image_height = data["imageHeight"]
        image_width = data["imageWidth"]
    yolo_annotations = []
    
    for shape in data['shapes']:
        label = shape['label']
        points = shape['points']
        label_to_id = {'graytote': 0, 'carton': 1} #change it according your dataset
        # Convert points to YOLO segmentation format
        
        yolo_points = []
        for point in points:
            x_normalized = point[0] / image_width
            y_normalized = point[1] / image_height
            yolo_points.append(f"{x_normalized} {y_normalized}")
        
        yolo_annotation = f"{label_to_id[label]} {' '.join(yolo_points)}"
        yolo_annotations.append(yolo_annotation)
    
    return yolo_annotations


files = glob.glob(input_path)
for json_file in files:
    name = Path(json_file).stem
    yolo_annotations = convert_json_to_yolo(json_file)
    with open(output_path + name + '.txt', 'w') as f:
        for annotation in yolo_annotations:
            f.write(annotation + '\n')
