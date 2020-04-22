class Region extends Type {
    constructor(name, points) {
        super(name);
        this.originalPoints = [];
        this.setOriginalPoints(points);
        this.translateX = 0;
        this.translateY = 0;
    }

    getPoints() {
        let translatedPoints = [];
        for (let i = 0; i < this.originalPoints.length; i++) {
            translatedPoints.push(new Point("",
                    this.originalPoints[i].getX() + this.translateX,
                    this.originalPoints[i].getY() + this.translateY));
        }
        return translatedPoints;
    }

    clear() {
        this.originalPoints = [];
    }

    setDeleted() {
        super.setDeleted();
        this.clear();
    }

    getOriginalPoints() {
        return this.originalPoints;
    }

    size() {
        return this.originalPoints.length;
    }

    setOriginalPoints(points) {
        this.clear();
        for (let i = 0; i < points.length; i++) {
            if (Array.isArray(points[i])) {
                this.originalPoints.push(new Point("", points[i][0], points[i][1]));
            } else {
                this.originalPoints.push(new Point("", points[i].getX(), points[i].getY()));
            }
        }
    }

    setOriginalPoint(pointId, newX, newY) {
        this.originalPoints[pointId].setCoordinate(newX, newY);
    }

    addOriginalPoint(x, y) {
        this.originalPoints.push(new Point("", x, y));
    }

    setTranslate(translateX, translateY) {
        this.translateX = translateX;
        this.translateY = translateY;
    }

    toString() {
        let originalPointsStr = "";
        for (let i = 0; i < this.originalPoints.length; i++) {
            originalPointsStr += "\n" + i + ": " + this.originalPoints[i].getX() + ", " + this.originalPoints[i].getY();
        }
        return super.toString() + 
                "\nTranslate (X, Y): (" + this.translateX + ", " + this.translateY + ")" + 
                "\nOriginal Points: " + originalPointsStr;
    }

    toHtmlString() {
        let originalPointsStr = "";
        for (let i = 0; i < this.originalPoints.length; i++) {
            originalPointsStr += "<p>" + i + ": " + 
                    this.originalPoints[i].getX() + ", " + this.originalPoints[i].getY() + "</p>";
        }
        return super.toHtmlString() + 
                "<p>Translate (X, Y): (" + this.translateX + ", " + this.translateY + ")</p>" +
                "<p>Original Points:</p>" + originalPointsStr;
    }
}