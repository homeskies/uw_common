class ChangeTracker {
    constructor() {
        this.pointTracker = new Map();  // point name -> Point
        this.poseTracker = new Map();   // pose name -> Pose
        this.regionTracker = new Map(); // region name -> Region
    }

    hasPoint(name) {
        return this.hasElement(this.pointTracker, name);
    }

    hasPose(name) {
        return this.hasElement(this.poseTracker, name);
    }

    hasRegion(name) {
        return this.hasElement(this.regionTracker, name);
    }

    applyPointChange(command, name, x=0, y=0, newName="") {
        // apply the command, return true if the command is valid, false otherwise
        var pointExist = this.pointTracker.has(name);
        if (command === "save") {
            if (pointExist) {
                var point = this.pointTracker.get(name);
                point.setCoordinate(x, y);
                point.setUndeleted();
            } else {
                this.pointTracker.set(name, new Point(name, x, y));
            }
            console.log("SAVE POINT");
            console.log(this.pointTracker.get(name).toString());
        } else if (command === "delete" && pointExist) {
            console.log("DELETE POINT");
            console.log("before: " + this.pointTracker.get(name).getDeleted());
            this.pointTracker.get(name).setDeleted();
            console.log("after: " + this.pointTracker.get(name).getDeleted());
        } else if (command === "rename" && pointExist && !this.hasPoint(newName)) {
            var point = this.pointTracker.get(name);
            var newPoint = new Point(newName, point.getX(), point.getY());
            this.renameElement(this.pointTracker, newPoint, point.getPrevName(), name, newName);
            console.log("RENAME POINT");
            console.log("prev point: " + this.pointTracker.has(name));
            console.log("new point: \n" + this.pointTracker.get(newName));
        } else {
            return false;
        }
        return true;
    }

    applyPoseChange(command, name, x=0, y=0, theta=0, newName="") {
        // apply the command, return true if the command is valid, false otherwise
        var poseExist = this.poseTracker.has(name);
        if (command === "save") {
            if (poseExist) {
                var pose = this.poseTracker.get(name);
                pose.setCoordinate(x, y, theta);
                pose.setUndeleted();
            } else {
                this.poseTracker.set(name, new Pose(name, x, y, theta));
            }
            console.log("SAVE POSE");
            console.log(this.poseTracker.get(name).toString());
        } else if (command === "delete" && poseExist) {
            console.log("DELETE POSE");
            console.log("before: " + this.poseTracker.get(name).getDeleted());
            this.poseTracker.get(name).setDeleted();
            console.log("after: " + this.poseTracker.get(name).getDeleted());
        } else if (command === "rename" && poseExist && !this.hasPose(newName)) {
            var pose = this.poseTracker.get(name);
            var newPose = new Pose(newName, pose.getX(), pose.getY(), pose.getTheta());
            this.renameElement(this.poseTracker, newPose, pose.getPrevName(), name, newName);
            console.log("RENAME POSE");
            console.log("prev pose: " + this.poseTracker.has(name));
            console.log("new pose: \n" + this.poseTracker.get(newName));
        } else {
            return false;
        }
        return true;
    }

    applyRegionChange(command, name, points=[], translateX=0, translateY=0, newName="") {
        // apply the command, return true if the command is valid, false otherwise
        var regionExist = this.regionTracker.has(name);
        if (command === "save") {
            if (regionExist) {
                var region = this.regionTracker.get(name);
                region.setOriginalPoints(points);
                region.setUndeleted();
            } else {
                var newRegion = new Region(name, points);
                this.regionTracker.set(name, newRegion);
            }
            console.log("SAVE REGION");
            console.log(this.regionTracker.get(name).toString());
        } else if (command === "translate" && regionExist) {
            this.regionTracker.get(name).setTranslate(translateX, translateY);
            console.log("TRANSLATE REGION");
            console.log(this.regionTracker.get(name).toString());
        } else if (command === "delete" && regionExist) {
            console.log("DELETE REGION");
            console.log("before: " + this.regionTracker.get(name).getDeleted());
            this.regionTracker.get(name).setDeleted();
            console.log("after: " + this.regionTracker.get(name).getDeleted());
        } else if (command === "rename" && regionExist && !this.hasRegion(newName)) {
            var region = this.regionTracker.get(name);
            var newRegion = new Region(newName, region.getOriginalPoints());
            this.renameElement(this.regionTracker, newRegion, region.getPrevName(), name, newName);
            console.log("RENAME REGION");
            console.log("prev region: " + this.regionTracker.has(name));
            console.log("new region: \n" + this.regionTracker.get(newName));
        } else {
            return false;
        }
        return true;
    }

    moveRegionEndpoint(regionName, pointId, newX, newY) {
        this.regionTracker.get(regionName).setOriginalPoint(pointId, newX, newY);
        console.log("MOVE REGION ENDPOINT");
        console.log("new region: \n" + this.regionTracker.get(regionName));
    }

    addRegionEndpoint(regionName, newX, newY) {
        this.regionTracker.get(regionName).addOriginalPoint(newX, newY);
        console.log("ADD REGION ENDPOINT");
        console.log("new region: \n" + this.regionTracker.get(regionName));
    }

    updateRegionEndpoints(regionName, newPoints) {
        var region = this.regionTracker.get(regionName);
        region.clear();
        region.setOriginalPoints(newPoints);
        console.log("UPDATE REGION ENDPOINT");
        console.log("new region: \n" + this.regionTracker.get(regionName));
    }

    hasElement(tracker, elementName) {
        // return true if the element exists, false otherwise
        var exist = tracker.has(elementName);
        if (exist && tracker.get(elementName).getDeleted()) {
            return false;
        }
        return exist;
    }

    renameElement(tracker, element, prevName, currentName, newName) {
        if (prevName != "") {
            element.setPrevName(prevName);
        } else {
            element.setPrevName(currentName);
        }
        tracker.delete(currentName);
        tracker.set(newName, element);
    }

    getChanges() {
        // return a string of all the tracked changes
        let str = "<h3>POINTS:</h3>";
        for (let [k, v] of this.pointTracker) {
            str += v.toHtmlString();
        }
        str += "<h3>POSES:</h3>";
        for (let [k, v] of this.poseTracker) {
            str += v.toHtmlString();
        }
        str += "<h3>REGION:</h3>";
        for (let [k, v] of this.regionTracker) {
            str += v.toHtmlString();
        }
        return str;
    }
}