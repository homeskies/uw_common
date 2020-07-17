import Type from './Type.js';

export default class Point extends Type {
    constructor(name, x, y) {
        super(name);
        this.x = x;
        this.y = y;
    }

    getX() {
        return this.x;
    }

    getY() {
        return this.y;
    }

    setCoordinate(x, y) {
        this.x = x;
        this.y = y;
    }

    toString() {
        return super.toString() + '\nX, Y: (' + this.x + ', ' + this.y + ')';
    }

    toHtmlString() {
        return super.toHtmlString() + '<p>X, Y: (' + this.x + ', ' + this.y + ')</p>';
    }
}