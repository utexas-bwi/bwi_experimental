// Initialize ROSBRIDGE connection
var ros;
function initializeROS()
{
   ros = new ROSLIB.Ros();

   // // If there is an error on the backend, an 'error' emit will be emitted.
   ros.on('error', function(error) {
     console.log('Error connecting to ' + rosBridgeAddress);
   });

   // Find out exactly when we made a connection.
   ros.on('connection', function() {
     console.log('Connection made to ' + rosBridgeAddress);
   });

   // // Create a connection to the rosbridge WebSocket server.
   ros.connect(rosBridgeAddress);
  
}


//SUBSCRIBE TO TOPICS
var sensorListeners = [];
var listenerCount = 0;
function createListener(topicname, messagetype, subtopicnames)
{
   // Create a Topic object with details of the topic's name and message type.
   sensorListeners[listenerCount] = new ROSLIB.Topic({
     ros : ros,
     name : topicname,
     messageType : messagetype
   });

   var JSONsubtopicnames = createJSONSubTopicArray(subtopicnames.slice(0));

   // Then we add a callback to be called every time a message is published on this topic.
   var currentCount = listenerCount;
   sensorListeners[listenerCount].subscribe(function(message) {

     document.getElementById('trUpdated' + currentCount).innerHTML = formatNowTimestamp();
     document.getElementById('trValue' + currentCount).innerHTML = '<div style="min-width:500px;" class="jsondiv">' + processListenerJSON(message, JSONsubtopicnames) + '</div>';

   });

   listenerCount++;

}

// Used as helper method to decide if we need to simplify our incoming topic or not
function processListenerJSON(jsonmessage, subtopicnames)
{
  //Check to see if the subtopics are empty
  var emptySubtopics = true;
  for(var i =0; i<subtopicnames.length && emptySubtopics; i++)
  {
    for(var j =0; j<subtopicnames[i].length && emptySubtopics; j++)
    {
      if(subtopicnames[i][j]!="")
      {
        emptySubtopics = false
      }
    }
  }

  if(emptySubtopics || subtopicnames.length==0)
  {
    // No subtopics to filter by, can just return the message
    return JSON.stringify(jsonmessage, null, 2);
  }
  else
  {
    // Need to simplify the message with our subtopics
    return JSON.stringify(getSimplifiedJSON(jsonmessage, subtopicnames.slice(0)), null, 2);
  }
}

//simplify JSON for sensor by getting rid of subtopics that aren't included in subtopicnames
function getSimplifiedJSON(jsonmessage, subtopicnames)
{
  if(subtopicnames.length==0)
  {
    return jsonmessage;
  }
  else
  {
    for(var attrname in jsonmessage)
    {
      var attrExists = false;

      for(var i=0; i<subtopicnames.length; i++)
      {
        if(subtopicnames[i][0]==attrname)
        {
          attrExists=true;
          break;
        }
      }

      if(!attrExists)
      {
        delete jsonmessage[attrname];
      }
    }

    var subtopicnamesshifted = shiftJSONSubTopicArray(subtopicnames.slice(0));
    for(var attrname in jsonmessage)
    {
      var attrFinal = false;
      for(var i=0; i<subtopicnames.length; i++)
      {
        if(subtopicnames[i][0]==attrname && subtopicnames[i].length==1)
        {
          attrFinal=true;
          break;
        }
      }
      if(!attrFinal)
      {
         jsonmessage[attrname] = getSimplifiedJSON(jsonmessage[attrname], subtopicnamesshifted.slice(0));
      }
    }

    return jsonmessage;
  }
}

//Adjust the subtopic arrays to point to the next node
function shiftJSONSubTopicArray(subtopicnames)
{
  for(var i = subtopicnames.length-1; i>=0; i--)
  {
    if(subtopicnames[i].length>1)
    {
      subtopicnames[i] = subtopicnames[i].slice(1);
    }
    else
    {
      subtopicnames.splice(i,1);
    }
  }
  return subtopicnames;
}

//create subtopic array that converts 'node1/node2/node3' to [node1,node2,node3]
function createJSONSubTopicArray(subtopicnames)
{
  for(var i=0; i<subtopicnames.length; i++)
  {
    subtopicnames[i] = subtopicnames[i].split("/");
  }
  return subtopicnames;
}

// Create the rows for the sensor table
function initializeSensorTable()
{
  var tbodySubscr = document.getElementById('tbodyTableSubscriptions');
    if (tableSubscriptions.length>0)
    {
      var counter = 0;
       tableSubscriptions.forEach(function(subscr) {

          var trSubscr = document.createElement('tr');
          trSubscr.id = 'trSubscr' + counter;

          var tdTopic = document.createElement('td');
          tdTopic.innerHTML = subscr.TopicName;

          var tdType = document.createElement('td');
          tdType.innerHTML = subscr.MessageType;
          tdType.className = 'hidden-xs hidden-sm';

          var tdUpdated = document.createElement('td');
          tdUpdated.id = 'trUpdated' + counter;

          var tdValue = document.createElement('td');
          tdValue.id = 'trValue' + counter;

          counter++;

          trSubscr.appendChild(tdTopic);
          trSubscr.appendChild(tdType);
          trSubscr.appendChild(tdUpdated);
          trSubscr.appendChild(tdValue);
          tbodySubscr.appendChild(trSubscr);
          
       });
    }
    else
    {
       var trSubscr = document.createElement('tr');
       trSubscr.id = 'trSubscrEMPTY';
       var tdEmpty = document.createElement('td');
       tdEmpty.colSpan = 4;
       tdEmpty.innerHTML = '<center>NO SUBSCRIPTONS</center>';
       trSubscr.appendChild(tdEmpty);
       tbodySubscr.appendChild(trSubscr);
    }
}

// Initialize subscriptions to log at the top of the dashboard
function initializeLogSubscriptions()
{
  logTopics.forEach(function(subscr) {
      var logListener = new ROSLIB.Topic({
         ros : ros,
         name : subscr.TopicName,
         messageType : subscr.MessageType
       });

       var JSONsubtopicnames = createJSONSubTopicArray(subscr.SubTopicNames.slice(0));

       // Then we add a callback to be called every time a message is published on this topic.
       logListener.subscribe(function(message) {
          logToScreen(subscr.TopicName, processListenerJSON(message, JSONsubtopicnames));
       });
   });
}

// Format timestamp for use in sensors and logs
function formatNowTimestamp() {
    var dt = new Date();

    var hours = dt.getHours();
    var minutes = dt.getMinutes();
    var seconds = dt.getSeconds();

    if (hours < 10) 
     hours = '0' + hours;

    if (minutes < 10) 
     minutes = '0' + minutes;

    if (seconds < 10) 
     seconds = '0' + seconds;

    return hours + ":" + minutes + ":" + seconds;
}  


// Create subscriptions for each of the topics we want to monitor in the sensors table
function createSensorSubscribers()
{
   listenerCount = 0;
   tableSubscriptions.forEach(function(subscr) {
      createListener(subscr.TopicName, subscr.MessageType, subscr.SubTopicNames);
   });
}

// Unsubscribe and delete listeners for current sensor topics
function deleteSensorSubscribers()
{
   var counter = 0;
   tableSubscriptions.forEach(function(subscr) {
      sensorListeners[counter].unsubscribe();
      sensorListeners[counter] = null;
      counter++;
   });
   listenerCount = 0;
}

// Initialize the video connection
var mjpegVideoViewer;
function createVideoSubscribers()
{
    // --------------------------------------------------------
    // Determine the size of the video that we can fit on the screen
    // --------------------------------------------------------
    var desiredVideoWidth = 640; //desired/max width
    var desiredVideoHeight = 480; //desired/max height

    var realVideoWidth;
    var realVideoHeight;

    //Current size of window
    var windowWidth = ("innerWidth" in window 
                ? window.innerWidth
                : document.documentElement.offsetWidth); 
    var windowHeight = ("innerHeight" in window 
             ? window.innerHeight
             : document.documentElement.offsetHeight); 

    //How big of a video can we fit for our width and height?
    var minVideoWidth = Math.min(desiredVideoWidth,  windowWidth);
    var minVideoHeight = Math.min(desiredVideoHeight, windowHeight);

    //If we resize our video according to the resolution we chose in config, 
    //what will we have to shift our videoWidth and videoHeight to respectively
    var shiftedVideoWidth = (minVideoHeight/desiredVideoHeight) * realVideoWidth;
    var shiftedVideoHeight = (minVideoWidth/desiredVideoWidth) * desiredVideoHeight;

    //check to see if Height takes priority over Width
    if(shiftedVideoWidth < minVideoWidth)
    {
       realVideoWidth = shiftedVideoWidth;
       realVideoHeight = minVideoHeight;
    }
    else
    {
       realVideoWidth = minVideoWidth;
       realVideoHeight = shiftedVideoHeight;
    }

    // --------------------------------------------------------

  // Create the main viewer.
  mjpegVideoViewer = new MJPEGCANVAS.MultiStreamViewer({
    divID : 'mjpegVideoScreen',
    host : rosMjpegServerInfo.Host,
    port : rosMjpegServerInfo.Port,
    quality : rosMjpegServerInfo.Quality,
    width : realVideoWidth,
    height : realVideoHeight,
    topics : rosMjpegServerInfo.Topics,
    labels : rosMjpegServerInfo.Labels
  });
}

// This doesn't erase the subscription, but it removes the viewer
function deleteVideoSubscribers()
{
    document.getElementById('mjpegVideoScreen').innerHTML = "";
    mjpegVideoViewer = null;
}

// Initialize the Navigation connection
var navWindowViewer;
var navWindowClient;
function createNavSubscribers()
{

    // --------------------------------------------------------
    // Determine the size of the Nav that we can fit on the screen
    // --------------------------------------------------------
    var desiredNavWidth = 800; //desired/max width
    var desiredNavHeight = 800; //desired/max height

    var realNavWidth;
    var realNavHeight;

    // Current size of window
    var windowWidth = ("innerWidth" in window 
                ? window.innerWidth
                : document.documentElement.offsetWidth)-25; 
    var windowHeight = ("innerHeight" in window 
             ? window.innerHeight
             : document.documentElement.offsetHeight)-25; 

    //How big of a Nav can we fit for our width and height?
    var minNavWidth = Math.min(desiredNavWidth, windowWidth);
    var minNavHeight = Math.min(desiredNavHeight, windowHeight);

    //If we resize our Nav according to the resolution we chose in config, 
    //what will we have to shift our NavWidth and NavHeight to respectively
    var shiftedNavWidth = (minNavHeight/desiredNavHeight) * desiredNavWidth;
    var shiftedNavHeight = (minNavWidth/desiredNavWidth) * desiredNavHeight;

    //check to see if Height takes priority over Width
    if(shiftedNavWidth < minNavWidth)
    {
       realNavWidth = shiftedNavWidth;
       realNavHeight = minNavHeight;
    }
    else
    {
       realNavWidth = minNavWidth;
       realNavHeight = shiftedNavHeight;
    }

    // Create the main viewer.
     navWindowViewer = new ROS2D.Viewer({
       divID : 'navWindow',
       width : realNavWidth,
       height : realNavHeight
     });

     // Setup the nav client.
     navWindowClient = NAV2D.OccupancyGridClientNav({
       ros : ros,
       rootObject : navWindowViewer.scene,
       viewer : navWindowViewer,
       serverName : navWindowServerName
     });

     //for normal map but without the location
     // navWindowClient = ROS2D.OccupancyGridClient({
     //   ros : ros,
     //   rootObject : navWindowViewer.scene
     // });

}

// This doesn't erase the subscription, but it removes the viewer
function deleteNavSubscribers()
{
    document.getElementById('navWindow').innerHTML = "";
    navWindowViewer = null;
}

// Initialize the teleop connection
var teleopClient;
function createTeleopSubscribers()
{
    // Create teleop client
    teleopClient = new KEYBOARDTELEOP.Teleop({
      ros : ros,
      topic : teleopTopic
    });

    // Create the slider for the speed
    $('#teleopSpeedSlider').slider({
      range : 'min',
      min : 0,
      max : 100,
      value : 100,
      slide : function(event, ui) {
        // Change the speed label.
        $('#teleopSpeedLabel').html('Speed: ' + ui.value + '%');
        // Scale the speed.
        teleopClient.scale = (ui.value / 100.0);
      }
    });

    // Set the initial speed
    $('#teleopSpeedLabel').html('Speed: ' + ($('#teleopSpeedSlider').slider('value')) + '%');
    teleopClient.scale = ($('#teleopSpeedSlider').slider('value') / 100.0);

}

// Removes teleop client
function deleteTeleopSubscribers()
{
    teleopClient.scale = 0;
    teleopClient = null;
}

// Carries out action for navigation buttons
function navAction(action)
{
  if(action=='left')
  {
    navWindowViewer.scene.x +=50;
  }
  else if(action=='right')
  {
    navWindowViewer.scene.x -=50;
  }
  else if(action=='down')
  {
    navWindowViewer.scene.y -=50;
  }
  else if(action=='up')
  {
    navWindowViewer.scene.y +=50;
  }
  else if(action=='zoomin')
  {
    // 1.5 is arbitrary, change the number to change how fast the zooming happens
    navWindowViewer.scene.scaleX = navWindowViewer.scene.scaleX*1.5;
    navWindowViewer.scene.scaleY = navWindowViewer.scene.scaleY*1.5;
  }
  else if(action=='zoomout')
  {
    // 1.5 is arbitrary, change the number to change how fast the zooming happens
    navWindowViewer.scene.scaleX = navWindowViewer.scene.scaleX/1.5;
    navWindowViewer.scene.scaleY = navWindowViewer.scene.scaleY/1.5;
  }
}


// initialize the drop downs for the connection/subscription defaults
function initializeConfigOptions()
{
  for(var index in connectionConfig) {
    $('#connectionSelect').append('<option value="' + index + '">' + index + '</option>');
  }

  for(var index in subscriptionConfig) {
    $('#subscriptionSelect').append('<option value="' + index + '">' + index + '</option>');
  }

  // Create empty fields for Custom entries
  addVideoTopic('');
  addSensorTopic('','',[]);
}

// Get url parameter for the id "sname"
function getParam ( sname )
{
  var params = location.search.substr(location.search.indexOf("?")+1);
  var sval = "";
  params = params.split("&");
    // split param and value into individual pieces
    for (var i=0; i<params.length; i++)
       {
         temp = params[i].split("=");
         if ( [temp[0]] == sname ) { sval = temp[1]; }
       }
  return sval;
}

// Process the url parameters (if any)
function processParamInitialization()
{
  //initialize to parameter values
  var connectionChoice = getParam("c");
  var subscriptionChoice = getParam("s");
  if(connectionChoice == "" && subscriptionChoice == "")
  {
    alert("ERROR: No connection or subscription configuration included.");
  }
  else if(connectionChoice == "")
  {
    alert("ERROR: No connection configuration included.");
  }
  else if(subscriptionChoice == "")
  {
    alert("ERROR: No subscription configuration included.");
  }
  else
  {
    //choices included
    if(!(connectionChoice in connectionConfig))
    {
      alert("ERROR: Connection configuration doesn't exist.");
    }
    else if(!(subscriptionChoice in subscriptionConfig))
    {
      alert("ERROR: Subscription configuration doesn't exist.");
    }
    else
    {
      //choices are valid, start the dashboard with the options chosen
      $("#connectionSelect").val(connectionChoice).trigger('change');
      $("#subscriptionSelect").val(subscriptionChoice).trigger('change');
      connectToROS();

    }
  }
}


// This is run when the page is started
function initialize()
{
  //redirect console output
  if (typeof console  != "undefined") 
  if (typeof console.log != 'undefined')
  {
      console.olog = console.log;
  }
  else
  {
      console.olog = function() {};
  }

  console.log = function(message) {
      console.olog(message);
      logToScreen('BWI Dashboard', message);
  };
  console.error = console.debug = console.info =  console.log

  //initialize config
  initializeConfigOptions();
  if(location.search.indexOf("?") >= 0)
  {
    // URL Parameters exist, process them
    processParamInitialization();
  }
  else
  {
    // No URL Parameters exist, carry on as normal
    $('#configurationcontainer').css({ "display": ''});
  }
}


