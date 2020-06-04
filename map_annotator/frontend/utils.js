function convertToString(points) {
    // convert a list of points to string in the form of: p1X,p1Y p2X,p2Y p3X,p3Y
    let stringOfPoints = "";
    for (let i = 0; i < points.length; i++) {
        let point = points[i];
        stringOfPoints += point[0] + "," + point[1] + " ";
    }
    return stringOfPoints.trim();
}

function convertToList(stringOfPoints) {
    // convert a string of points in the form of: p1X,p1Y p2X,p2Y p3X,p3Y to a list of points
    let points = [];
    let stringSplitted = stringOfPoints.trim().split(" ");
    for (let i = 0; i < stringSplitted.length; i++) {
        let pointStr = stringSplitted[i].split(",");
        points.push([parseInt(pointStr[0]), parseInt(pointStr[1])]);
    }
    return JSON.parse(JSON.stringify(points));
}

function getRandomInteger(min, max) {
    return Math.floor(Math.random() * (max - min)) + min;
}

function getTransformMatrix(transformMatrixStr) {
    // return the transform matrix as a list
    let transformMatrix = transformMatrixStr.substring(7, transformMatrixStr.length - 1);
    let transformMatrixArr = transformMatrix.split(",");
    let result = [];
    for (let i = 0; i < transformMatrixArr.length; i++) {
        result.push(parseFloat(transformMatrixArr[i]));
    }
    return result;
}

function getTranslate(translateStr) {
    // return the translate in the form of: [x, y]
    let translate = translateStr.substring(10, translateStr.length - 1);
    let translateArr = translate.split(",");
    return [parseFloat(translateArr[0]), parseFloat(translateArr[1])];
}

function getRegionId(selectedRegion) {
    // return the region id of the selected region
    let referencePointElement = selectedRegion.parentElement.childNodes[2];
	return parseInt(referencePointElement.getAttribute('id').split("-")[0]);
}

function getLabelElement(element) {
    // given a shape element, return the label of it
    return element.parentElement.childNodes[0];
}

function convertToDeg(radian) {
    // convert radian to degree
    return radian * (180 / Math.PI);
}

function round(num) {
    // roung the number to 2 decimal places
    return Math.round(num * 100) / 100;
}

function promptForName(defaultLabelName) {
    // prompt for the user to enter an element name
    // return the user input if the input is nonempty, return "" otherwise
    let labelName = prompt("Please enter the label name:", defaultLabelName);
    if (labelName == null) {
        return "";
    }
    if (labelName === "") {
        alert("Please enter a valid name!");
    }
    return labelName;
}

function showPopup(helpPopup, helpPopupContent, disableDiv, content) {
    // display the pop up window
    helpPopup.style.display = "block";
    helpPopupContent.innerHTML = content;
    // disable everything in the background
    disableDiv.style.display = "block";
}