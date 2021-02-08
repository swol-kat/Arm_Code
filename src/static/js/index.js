window.addEventListener("DOMContentLoaded", main);

var socket;

function main() {
  //plot setup
  setup_plot();

  socket = io("ws://localhost:6969");
  // alert when ready

  // handle the event sent with socket.send()
  socket.on("update", update_data);

  setup_intervals();
  setup_jogs();
  
}

function setup_intervals() {
  window.setInterval(function () {
    socket.emit("get_update");
  }, 100);
}


function setup_jogs(){
  document.getElementById('jog_container').childNodes.forEach(set_jog_dir)
}

function set_jog_dir(el){
  if (el.tagName == 'DIV' && el.dataset.axis != null) {
  el.childNodes[1].onclick = () => socket.emit('jog', {
    amount: document.getElementById('jog_amount').value,
    axis: el.dataset.axis
  })
  el.childNodes[3].onclick = () => socket.emit('jog', {
    amount: - document.getElementById('jog_amount').value,
    axis: el.dataset.axis
  })
}
}


function update_data(d) {
  update_plot(d.joint_pos.x, d.joint_pos.y, d.joint_pos.z);
  delete d.joint_pos;
  document.getElementById("status").innerHTML = JSON.stringify(d, null, 2);
}

function setup_plot() {
  //setting up plot

  x = [0];
  y = [0];
  z = [0];
  var data = [
    {
      type: "scatter3d",
      mode: "lines+markers",
      x: x,
      y: y,
      z: z,
      marker: {
        size: 3,
      },
    },
  ];

  var layout = {
    scene: {
      aspectratio: {
        x: 1,
        y: 1,
        z: 1,
      },
      xaxis: {
        range: [-20, 20],
      },
      yaxis: {
        range: [-20, 20],
      },
      zaxis: {
        range: [0, 20],
      },
    },
  };

  var robot_plot = Plotly.newPlot("robot_plot", data, layout);
}

function update_plot(x, y, z) {
  var data = {
    x: [x],
    y: [y],
    z: [z],
  };
  Plotly.restyle("robot_plot", data);
}
