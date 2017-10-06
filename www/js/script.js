$(document).ready(function(){
  $("#ArrowUp").bind('touchstart', function(){sendToSocket("UP");});
  $("#ArrowDown").bind('touchstart', function(){sendToSocket("DOWN");});
  $("#ArrowLeft").bind('touchstart', function(){sendToSocket("LEFT");});
  $("#ArrowRight").bind('touchstart', function(){sendToSocket("RIGHT");});
  $("#ev3logo").bind('touchstart', function(){sendToSocket("CLR");});
  $("#ArrowUp").vmousedown(function(){sendToSocket("UP");});
  $("#ArrowDown").vmousedown(function(){sendToSocket("DOWN");});
  $("#ArrowLeft").vmousedown(function(){sendToSocket("LEFT");});
  $("#ArrowRight").vmousedown(function(){sendToSocket("RIGHT");});
  $("#ev3logo").vmousedown(function(){sendToSocket("CLR");});
});
function(direction){
	var host = document.getElementById('ipaddress').value;
	var port = 12000;
	var socket = new jSocket();
	socket.onReady = function(){
		socket.connect(host, port);
	}
	socket.onConnect = function(success, msg){
		if(success){
			socket.write(direction);
		}else{
			alert("Cannot connect to EV3: " + msg);
		}
	}
	socket.onData = function(data){
		alert('Connection from socket: ' + data);
	}
}
