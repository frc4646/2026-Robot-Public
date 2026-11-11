import json
import os


def flipRot(rotation):
    return rotation * -1


def flipPath(path):
    waypoints = path["waypoints"]
    for idx in range(len(waypoints)):
        path["waypoints"][idx]["linkedName"] = None
        path["waypoints"][idx]["anchor"]["y"] = (
            8 - path["waypoints"][idx]["anchor"]["y"]
        )
        if path["waypoints"][idx]["nextControl"]:
            path["waypoints"][idx]["nextControl"]["y"] = (
                8 - path["waypoints"][idx]["nextControl"]["y"]
            )
        if path["waypoints"][idx]["prevControl"]:
            path["waypoints"][idx]["prevControl"]["y"] = (
                8 - path["waypoints"][idx]["prevControl"]["y"]
            )

    rotationTargets = path["rotationTargets"]
    for idx in range(len(rotationTargets)):
        path["rotationTargets"][idx]["rotationDegrees"] = flipRot(
            path["rotationTargets"][idx]["rotationDegrees"]
        )
    path["idealStartingState"]["rotation"] = flipRot(
        path["idealStartingState"]["rotation"]
    )

    path["goalEndState"]["rotation"] = flipRot(path["goalEndState"]["rotation"])
    if not (path["folder"] + " Flipped") in settings["pathFolders"]:
        settings["pathFolders"].append(path["folder"] + " Flipped")
    path["folder"] = path["folder"] + " Flipped"


def flipAuto(auto):
    print(auto)
    for command in auto["data"]["commands"]:
        print(command)
        if command["type"] == "path":
            command["data"]["pathName"] = (
                command["data"]["pathName"].split(".")[0] + " flipped"
            )
        elif (
            "parallel" in command["type"]
            or "sequential" in command["type"]
            or "race" in command["type"]
            or "deadline" in command["type"]
        ):
            print("flipping: ", command)
            flipAuto(command)
        elif (
            "named" in command["type"]
            and "Drive to Right Branch" in command["data"]["name"]
        ):
            command["data"]["name"] = "Drive to Left Branch"
        elif (
            "named" in command["type"]
            and "Drive to Left Branch" in command["data"]["name"]
        ):
            command["data"]["name"] = "Drive to Right Branch"


pathplannerPath = r"./src/main/deploy/pathplanner/"
settingsFile = open(r"./src/main/deploy/pathplanner/settings.json")
settings = json.load(settingsFile)

pathsDir = pathplannerPath + "paths/"
for filename in os.listdir(pathsDir):
    with open(pathsDir + filename) as file:
        data = json.load(file)
    if data["folder"] is None or "flipped" in filename:
        continue
    destination = open(pathsDir + filename.split(".")[0] + " flipped.path", mode="w")
    flipPath(data)
    json.dump(data, destination, indent=2)


autosDir = pathplannerPath + "autos/"
for filename in os.listdir(autosDir):
    autoFile = open(pathplannerPath + "autos/" + filename)
    auto = json.load(autoFile)
    if auto["folder"] is not None or "flipped" in filename:
        continue
    flipAuto(auto["command"])
    flippedAutoFile = open(
        pathplannerPath + "autos/" + filename.split(".")[0] + " flipped.auto", mode="w"
    )
    json.dump(auto, flippedAutoFile, indent=2)


settingsFileOut = open(r"./src/main/deploy/pathplanner/settings.json", mode="w")
json.dump(settings, settingsFileOut, indent=2)
