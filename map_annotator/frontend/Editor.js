class Editor {
	constructor() {
		this.svg = null;
		this.width = "";
		this.height = "";
	}

	getMidpointX() {
		return parseFloat(this.width) / 2;
	}

	getMidpointY() {
		return parseFloat(this.height) / 2;
	}

	setSVG(svg, svgFileContent) {	
		this.svg = svg;
		this.width = svgFileContent.documentElement.viewBox.baseVal.width.toString();
		this.height = svgFileContent.documentElement.viewBox.baseVal.height.toString();
		this.svg.setAttribute("width", this.width);
		this.svg.setAttribute("height", this.height);
		this.svg.setAttribute("viewBox", "0 0 " + this.width + " " + this.height);
		this.svg.innerHTML = svgFileContent.documentElement.innerHTML;
	}

	addElement(element) {
		this.svg.appendChild(element);
		this.svg.appendChild(document.createTextNode('\n'));
	}

	deleteElement(element) {
		this.svg.removeChild(element);
	}

	deleteElementOfGroup(g, element) {
		g.removeChild(element);
	}

	clear() {
		this.svg.textContent = '';
	}

	toString() {
		return [
			'<?xml version="1.0" encoding="UTF-8"?>\n',
			'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ' + this.width + ' ' + this.height + '\">\n',
			this.svg.innerHTML,
			'</svg>'
		].join('');
	}

}
