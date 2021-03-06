var PATHS = new Array();
var PATHS_CANVAS_CONTEXT = null;

var PATHS_HORIZONTAL = new Array();
var PATHS_VERTICAL = new Array();
/*
PATHS define where pac man and the ghosts can move to
*/

function initPaths() {
	var canvas = document.getElementById('canvas-paths');
	canvas.setAttribute('width', '550');
	canvas.setAttribute('height', '550');
	if (canvas.getContext) {
		PATHS_CANVAS_CONTEXT = canvas.getContext('2d');
	}

	// CENTER
	PATHS.push("128,416-422,416");
	PATHS.push("30,98-518,98");
	PATHS.push("10,258-186,258");//teleport Left
	PATHS.push("362,258-540,258"); //teleport right
	PATHS.push("186,204-362,204");
	PATHS.push("186,310-362,310");
	PATHS.push("30,522-518,522");
	PATHS.push("238,258-314,258");
	PATHS.push("276,204-276,258");

	// LEFT
	PATHS.push("128,26-128,470");//V
	PATHS.push("30,26-244,26");//H
	PATHS.push("30,26-30,150");//V
	PATHS.push("30,150-128,150");//H
	PATHS.push("244,26-244,98");//V
	PATHS.push("186,204-186,364");//V
	PATHS.push("30,364-244,364");//H

	PATHS.push("244,364-244,416");//V
	PATHS.push("30,364-30,416");//V
	PATHS.push("30,416-70,416");//H
	PATHS.push("70,416-70,470");//V
	PATHS.push("30,470-128,470");//H
	PATHS.push("30,470-30,522");//V
	PATHS.push("244,150-244,204");//V
	PATHS.push("186,150-244,150");//H
	PATHS.push("186,98-186,150");//V
	PATHS.push("244,470-244,522");//V
	PATHS.push("186,470-244,470");//H
	PATHS.push("186,416-186,470");//V

	// RIGHT
	PATHS.push("422,26-422,470");//V
	PATHS.push("304,26-518,26");//H
	PATHS.push("518,26-518,150");//V
	PATHS.push("304,26-304,98");//V
	PATHS.push("422,150-518,150");//H
	PATHS.push("362,204-362,364");//V
	PATHS.push("304,364-518,364");//H
	PATHS.push("304,364-304,416");//V
	PATHS.push("518,364-518,416");//V
	PATHS.push("480,416-518,416");//H
	PATHS.push("480,416-480,470");//V
	PATHS.push("422,470-518,470");//H
	PATHS.push("518,470-518,522");//V
	PATHS.push("304,150-304,204");//V
	PATHS.push("304,150-362,150");//H
	PATHS.push("362,98-362,150");//V
	PATHS.push("304,470-304,522");//V
	PATHS.push("304,470-362,470");//H
	PATHS.push("362,416-362,470");//V

	//VERTICAL
	PATHS_VERTICAL.push("276,204-276,258");
	//LEFT
	PATHS_VERTICAL.push("128,26-128,470");
	PATHS_VERTICAL.push("30,26-30,150");
	PATHS_VERTICAL.push("244,26-244,98");
	PATHS_VERTICAL.push("186,204-186,364");
	PATHS_VERTICAL.push("244,364-244,416");//V
	PATHS_VERTICAL.push("30,364-30,416");//V
	PATHS_VERTICAL.push("70,416-70,470");//V
	PATHS_VERTICAL.push("30,470-30,522");//V
	PATHS_VERTICAL.push("244,150-244,204");//V
	PATHS_VERTICAL.push("186,98-186,150");//V
	PATHS_VERTICAL.push("244,470-244,522");//V
	PATHS_VERTICAL.push("186,416-186,470");//V
	//RIGHT
	PATHS_VERTICAL.push("422,26-422,470");//V
	PATHS_VERTICAL.push("518,26-518,150");//V
	PATHS_VERTICAL.push("304,26-304,98");//V
	PATHS_VERTICAL.push("362,204-362,364");//V
	PATHS_VERTICAL.push("304,364-304,416");//V
	PATHS_VERTICAL.push("518,364-518,416");//V
	PATHS_VERTICAL.push("480,416-480,470");//V
	PATHS_VERTICAL.push("518,470-518,522");//V
	PATHS_VERTICAL.push("304,150-304,204");//V
	PATHS_VERTICAL.push("362,98-362,150");//V
	PATHS_VERTICAL.push("304,470-304,522");//V
	PATHS_VERTICAL.push("362,416-362,470");//V

	//HORIZONTAL
	PATHS_HORIZONTAL.push("128,416-422,416");
	PATHS_HORIZONTAL.push("30,98-518,98");
	PATHS_HORIZONTAL.push("10,258-186,258");//teleport Left
	PATHS_HORIZONTAL.push("362,258-540,258"); //teleport right
	PATHS_HORIZONTAL.push("186,204-362,204");
	PATHS_HORIZONTAL.push("186,310-362,310");
	PATHS_HORIZONTAL.push("30,522-518,522");
	PATHS_HORIZONTAL.push("238,258-314,258");
	//LEFT
	PATHS_HORIZONTAL.push("30,26-244,26");
	PATHS_HORIZONTAL.push("30,150-128,150");
	PATHS_HORIZONTAL.push("30,364-244,364");
	PATHS_HORIZONTAL.push("30,416-70,416");//H
	PATHS_HORIZONTAL.push("30,470-128,470");//H
	PATHS_HORIZONTAL.push("186,150-244,150");//H
	PATHS_HORIZONTAL.push("186,470-244,470");//H
	//RIGHT
	PATHS_HORIZONTAL.push("304,26-518,26");//H
	PATHS_HORIZONTAL.push("422,150-518,150");//H
	PATHS_HORIZONTAL.push("304,364-518,364");//H
	PATHS_HORIZONTAL.push("480,416-518,416");//H
	PATHS_HORIZONTAL.push("422,470-518,470");//H
	PATHS_HORIZONTAL.push("304,150-362,150");//H
	PATHS_HORIZONTAL.push("304,470-362,470");//H
}

function getPathsCanevasContext() {
	return PATHS_CANVAS_CONTEXT;
}

function drawPaths() {
	var ctx = getPathsCanevasContext();

	ctx.strokeStyle = "red";

	for (var i = 0, imax = PATHS.length; i < imax; i ++) {

		var p = PATHS[i];

		var startX = p.split("-")[0].split(",")[0];
		var startY = p.split("-")[0].split(",")[1];
		var endX = p.split("-")[1].split(",")[0];
		var endY = p.split("-")[1].split(",")[1];

		ctx.beginPath();
		ctx.moveTo(startX, startY);
		ctx.lineTo(endX, endY);
		ctx.stroke();
		ctx.closePath();
	}
}
