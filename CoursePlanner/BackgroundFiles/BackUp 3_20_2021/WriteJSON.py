import json

jsonData = {}

jsonData["Constants"] = []
jsonData["Constants"].append({
    "k_searchInterval": "12",
    "k_lineThickness": "10",
    "k_markerRadius": "2",
    "k_orginColor": [0, 255, 0],
    "k_markerColor": [255, 40, 5], 
    "k_waypointStringingRadius": "43",
    "k_pixelScalingFactor": "009208",
    "k_maxPathSmoothingDistance": "100",
    "k_convertPathToCode": "True",
    "k_initalXOffset": "0",
    "k_initialYOffset": "0"
})

jsonData["FileNaming"] = []
jsonData["FileNaming"].append({
    "f_textOutputName": "outputPath.txt",
    "f_imageOutputName": "outputImage.jpg",
    "f_baseImageName": "BaseImage.jpg",
    "f_imageInputName": "NewImage6.jpg"
})

jsonFile = open("data.txt","w")
json.dump(jsonData, jsonFile)