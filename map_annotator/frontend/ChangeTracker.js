class ChangeTracker {
    constructor(mapAnnotationTopic, editor) {
        // trackers
        this.mapName = "";
        this.pointTracker = new Map();  // point name -> Point
        this.poseTracker = new Map();   // pose name -> Pose
        this.regionTracker = new Map(); // region name -> Region
        // ROS topic
        this.mapAnnotationTopic =mapAnnotationTopic;
        // editor
        this.editor = editor;
    }

    setMapName(mapName) {
        this.mapName = mapName;
    }

    getMapName() {
        return this.mapName;
    }

    reset() {
        this.pointTracker.clear();
        this.poseTracker.clear();
        this.regionTracker.clear();
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
        let pointExist = this.pointTracker.has(name);
        if (command === "save") {
            if (pointExist) {
                let point = this.pointTracker.get(name);
                point.setCoordinate(x, y);
                point.setUndeleted();
            } else {
                this.pointTracker.set(name, new Point(name, x, y));
            }
        } else if (command === "delete") {
            if (pointExist) {
                this.pointTracker.get(name).setDeleted();
            } else {
                let point = new Point(name, null, null);
                point.setDeleted();
                this.pointTracker.set(name, point);
            }
        } else if (command === "rename") {
            if (pointExist && !this.hasPoint(newName)) {
                let point = this.pointTracker.get(name);
                let newPoint = new Point(newName, point.getX(), point.getY());
                this.renameElement(this.pointTracker, newPoint, point.getPrevName(), name, newName);
            } else if (newName != name) {
                let newPoint = new Point(newName, null, null);
                newPoint.setPrevName(name);
                this.pointTracker.set(newName, newPoint);
            }
        }
    }

    applyPoseChange(command, name, x=0, y=0, theta=0, newName="") {
        let poseExist = this.poseTracker.has(name);
        if (command === "save") {
            if (poseExist) {
                let pose = this.poseTracker.get(name);
                pose.setCoordinate(x, y, theta);
                pose.setUndeleted();
            } else {
                this.poseTracker.set(name, new Pose(name, x, y, theta));
            }
        } else if (command === "delete") {
            if (poseExist) {
                this.poseTracker.get(name).setDeleted();
            } else {
                let pose = new Pose(name, null, null, null);
                pose.setDeleted();
                this.poseTracker.set(name, pose);
            }
        } else if (command === "rename") {
            if (poseExist && !this.hasPose(newName)) {
                let pose = this.poseTracker.get(name);
                let newPose = new Pose(newName, pose.getX(), pose.getY(), pose.getTheta());
                this.renameElement(this.poseTracker, newPose, pose.getPrevName(), name, newName);
            } else if (newName != name) {
                let newPose = new Pose(newName, null, null, null);
                newPose.setPrevName(name);
                this.poseTracker.set(newName, newPose);
            }
        }
    }

    applyRegionChange(command, name, points=[], translateX=0, translateY=0, newName="") {
        let regionExist = this.regionTracker.has(name);
        if (command === "save") {
            if (regionExist) {
                let region = this.regionTracker.get(name);
                region.setOriginalPoints(points);
                region.setUndeleted();
            } else {
                let newRegion = new Region(name, points);
                this.regionTracker.set(name, newRegion);
            }
        } else if (command === "translate") {
            if (regionExist) {
                let region = this.regionTracker.get(name);
                region.setTranslate(translateX, translateY);
                if (points != null) {
                    region.setOriginalPoints(points);
                }
            } else {
                let newRegion = new Region(name, []);
                newRegion.setTranslate(translateX, translateY);
                if (points != null) {
                    newRegion.setOriginalPoints(points);
                }
                this.regionTracker.set(name, newRegion);
            }
        } else if (command === "delete") {
            if (regionExist) {
                this.regionTracker.get(name).setDeleted();
            } else {
                let region = new Region(name, []);
                region.setDeleted();
                this.regionTracker.set(name, region);
            }
        } else if (command === "rename") {
            if (regionExist && !this.hasRegion(newName)) {
                let region = this.regionTracker.get(name);
                let newRegion = new Region(newName, region.getOriginalPoints());
                this.renameElement(this.regionTracker, newRegion, region.getPrevName(), name, newName);
            } else if (newName != name) {
                let newRegion = new Region(newName, []);
                newRegion.setPrevName(name);
                this.regionTracker.set(newName, newRegion);
            }
        }
    }

    moveRegionEndpoint(regionName, pointId, newX, newY) {
        if (this.regionTracker.has(regionName)) {
            let region = this.regionTracker.get(regionName);
            if (region.size() > pointId) {
                region.setOriginalPoint(pointId, newX, newY);
            } else {
                this.createEndpoints(pointId - region.size(), region, newX, newY);
            }
        } else {
            this.createRegionAndEndpoints(regionName, pointId, newX, newY);
        }
    }

    addRegionEndpoint(regionName, pointId, newX, newY) {
        if (this.regionTracker.has(regionName)) {
            let region = this.regionTracker.get(regionName);
            this.createEndpoints(pointId - region.size(), region, newX, newY);
        } else {
            this.createRegionAndEndpoints(regionName, pointId, newX, newY);
        }
    }

    updateRegionEndpoints(regionName, newPoints) {
        if (this.regionTracker.has(regionName)) {
            let region = this.regionTracker.get(regionName);
            region.clear();
            region.setOriginalPoints(newPoints);
        } else {
            let newRegion = new Region(regionName, newPoints);
            this.regionTracker.set(regionName, newRegion);
        }
    }

    hasElement(tracker, elementName) {
        // return true if the element exists, false otherwise
        let exist = tracker.has(elementName);
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

    createEndpoints(num, region, newX, newY) {
        for (let i = 0; i < num; i++) {
            region.addOriginalPoint(null, null);
        }
        region.addOriginalPoint(newX, newY);
    }

    createRegionAndEndpoints(regionName, pointId, newX, newY) {
        let newRegion = new Region(regionName, []);
        for (let i = 0; i < pointId; i++) {
            newRegion.addOriginalPoint(null, null);
        }
        newRegion.addOriginalPoint(newX, newY);
        this.regionTracker.set(regionName, newRegion);
    }

    publishChanges(prevName, currentName) {
        let points = [];
        for (let [k, v] of this.pointTracker) {
            points.push(this.getPointMsg(v));
        }
        let poses = [];
        for (let [k, v] of this.poseTracker) {
            poses.push(this.getPoseMsg(v));
        }
        let regions = [];
        for (let [k, v] of this.regionTracker) {
            let endpoints = v.getPoints();
            let endpointsMsg = [];
            for (let ep of endpoints) {
                endpointsMsg.push(this.getPointMsg(ep));
            }
            regions.push(new ROSLIB.Message({
                prev_name: v.getPrevName(),
                current_name: v.getName(),
                deleted: v.getDeleted(),
                endpoints: endpointsMsg
            }));
        }
        let msg = new ROSLIB.Message({
            prev_name: prevName,
            current_name: currentName,
            points: points,
            poses: poses,
            regions: regions
        });
        this.mapAnnotationTopic.publish(msg);
    }

    getPointMsg(point) {
        // convert to map coordinates
        let mapCoordinate = this.editor.getMapCoordinate(point.getX(), point.getY());
        return new ROSLIB.Message({
            prev_name: point.getPrevName(),
            current_name: point.getName(),
            deleted: point.getDeleted(),
            x: mapCoordinate[0],
            y: mapCoordinate[1]
        });
    }

    getPoseMsg(pose) {
        // convert to map coordinates
        let mapCoordinate = this.editor.getMapCoordinate(pose.getX(), pose.getY());
        return new ROSLIB.Message({
            prev_name: pose.getPrevName(),
            current_name: pose.getName(),
            deleted: pose.getDeleted(),
            x: mapCoordinate[0],
            y: mapCoordinate[1],
            theta: -pose.getTheta()
        });
    }
}