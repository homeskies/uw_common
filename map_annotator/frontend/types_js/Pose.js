import Point from './Point.js';

export default class Pose extends Point {
    constructor(name, x, y, theta) {
        super(name, x, y);
        this.theta = theta;
    }

    getTheta() {
        return this.theta;
    }

    setCoordinate(x, y, theta) {
        super.setCoordinate(x, y);
        this.theta = theta;
    }

    toString() {
        return super.toString() + '\nTheta: ' + (-this.theta);
    }

    toHtmlString() {
        return super.toHtmlString() + '<p>Theta: ' + (-this.theta) + '</p>';
    }
}