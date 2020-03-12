class Type {
    constructor(name) {
        this.name = name;
        this.deleted = false;
        // the name stored in the database at the time the svg file is loaded
        this.prevName = "";
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
        this.prevName = (this.name != prevName) ? this.prevName = prevName : "";
    }

    toString() {
        return "Name: " + this.name + "\nDeleted: " + this.deleted +
               "\nPrevName: " + this.prevName;
    }

    toHtmlString() {
        return "<p>Name: " + this.name + "</p><p>Deleted: " + this.deleted +
                "</p><p>PrevName: " + this.prevName + "</p>";
    }
}