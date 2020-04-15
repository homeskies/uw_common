class Type {
    constructor(name) {
        this.name = name;
        this.deleted = false;
        // the name stored in the database at the time the svg file is loaded
        this.prevName = "";
    }

    getName() {
        return this.name;
    }

    getPrevName() {
        return this.prevName;
    }

    getDeleted() {
        return this.deleted;
    }

    setDeleted() {
        this.deleted = true;
    }

    setUndeleted() {
        this.deleted = false;
    }

    setPrevName(prevName) {
        this.prevName = (this.name != prevName) ? prevName : "";
    }

    toString() {
        return "Name: " + this.name + "\nDeleted: " + this.deleted +
               "\nPrevious Name: " + this.prevName;
    }

    toHtmlString() {
        return "<p>Name: " + this.name + "</p><p>Deleted: " + this.deleted +
                "</p><p>Previous Name: " + this.prevName + "</p>";
    }
}