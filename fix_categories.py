import json

with open("_annotations.coco.json") as f:
    data = json.load(f)

# Replace categories with just one
data["categories"] = [{"id": 1, "name": "hand", "supercategory": "none"}]

# Remap all annotations: categories 1,2,3,4 → 1, drop category 0 (parent "hands" has no boxes)
new_annotations = []
for ann in data["annotations"]:
    if ann["category_id"] in [1, 2, 3, 4]:
        ann["category_id"] = 1
        new_annotations.append(ann)

data["annotations"] = new_annotations

with open("_annotations_fixed.coco.json", "w") as f:
    json.dump(data, f, indent=4)

print(f"Done: {len(data['images'])} images, {len(data['annotations'])} annotations")
