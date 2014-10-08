<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no" />
    <meta name="description" content="">
    <meta name="author" content="">
    <link rel="shortcut icon" href="ico/favicon.ico">

    <title>BWI Web Tool</title>

    <!-- Bootstrap core CSS -->
    <link href="css/bootstrap.min.css" rel="stylesheet">
    <link rel="stylesheet" type="text/css" href="http://ajax.googleapis.com/ajax/libs/jqueryui/1.8/themes/base/jquery-ui.css" />

    <!-- Custom styles for this style -->
    <link href="css/dashboard.css" rel="stylesheet">

    <!-- Import necessary javascript files -->
    <script src="js/easeljs.min.js"></script>
    <script src="js/eventemitter2.js"></script>
    <script src="js/roslib.js"></script>
    <script src="js/ros2d.min.js"></script>
    <script src="js/nav2d.min.js"></script>
    <script src="js/jquery.min.js"></script>
    <script src="js/jquery-ui.min.js"></script>
    <script src="js/bootstrap.min.js"></script>
    <script src="js/mjpegcanvas.min.js"></script>
    <script src="js/CustomKeyboardTeleop.js"></script>
    <script src="js/docs.min.js"></script>
    <script src="js/config.js"></script>
    <script src="js/dashboard.js"></script>

  </head>

  <body onload="initialize()">

  <!-- HEADER -->
    <div class="navbar navbar-inverse" role="navigation" style="margin-top:-50px;">
      <div class="container-fluid">
        <div class="navbar-header">
          <button type="button" class="navbar-toggle" data-toggle="collapse" data-target=".navbar-collapse">
            <span class="sr-only">Toggle navigation</span>
            <span class="icon-bar"></span>
            <span class="icon-bar"></span>
            <span class="icon-bar"></span>
          </button>
          <a class="navbar-brand" href="#">BWI Web Tool</a>
        </div>
        <div class="navbar-collapse collapse">
          <ul class="nav navbar-nav navbar-right">
            <li><a href="#">Dashboard</a></li>
            <!-- <li><a href="#">Settings</a></li> -->
            <!-- <li><a href="#">Profile</a></li> -->
            <!-- <li><a href="#">Help</a></li> -->
          </ul>
        </div>
      </div>
    </div>


    <div class="container-fluid">

    <!-- CONFIGURATION SCREEN -->
      <div id="configurationcontainer" style="display:none;">
        <h1 class="page-header">Configuration</h1>

        <div class="panel panel-default">
          <div class="panel-heading clearfix">
            <h3 class="panel-title pull-left">Connection</h3>
            <div class="btn-group pull-right">
              <button type="button" class="btn btn-default btn-xs" id="connectionExport">
                <span class="glyphicon glyphicon-download-alt"></span> Export Configuration
              </button>
            </div>
          </div>
          <div class="panel-body">
            <div class="input-group">
              <span class="input-group-addon">Select Default</span>
              <select class="form-control" id="connectionSelect">
                <option value="CUSTOM">Custom</option>
              </select>
            </div>
            <div class="input-group">
              <span class="input-group-addon">Host</span>
              <input type="text" class="form-control" placeholder="example.csres.utexas.edu" id="inputHost">
            </div>
            <div class="input-group">
              <span class="input-group-addon">Rosbridge Port</span>
              <input type="text" class="form-control" placeholder="9090" id="inputRosbridgePort">
            </div>
            <div class="input-group">
              <span class="input-group-addon">Mjpeg Server Port</span>
              <input type="text" class="form-control" placeholder="8080" id="inputMjpegServerPort">
            </div>
          </div>
        </div>

        <div class="panel panel-default">
          <div class="panel-heading clearfix">
            <h3 class="panel-title pull-left">Subscriptions</h3>
            <div class="btn-group pull-right">
              <button type="button" class="btn btn-default btn-xs" id="subscriptionExport">
                <span class="glyphicon glyphicon-download-alt"></span> Export Configuration
              </button>
            </div>
          </div>
          <div class="panel-body">
            <div class="input-group">
              <span class="input-group-addon">Select Default</span>
              <select class="form-control" id="subscriptionSelect">
                <option value="CUSTOM">Custom</option>
              </select>
            </div>

            <h3 class="page-header">Enabled Widgets</h2>
            <div class="input-group">
              <span class="input-group-addon">
                <input type="checkbox" id="enabledVideo" checked>
              </span>
              <input type="text" class="form-control" value="Video" readonly="readonly">
            </div>
            <div class="input-group">
              <span class="input-group-addon">
                <input type="checkbox" id="enabledTeleop" checked>
              </span>
              <input type="text" class="form-control" value="Teleop" readonly="readonly">
            </div>
            <div class="input-group">
              <span class="input-group-addon">
                <input type="checkbox" id="enabledNavigation" checked>
              </span>
              <input type="text" class="form-control" value="Navigation" readonly="readonly">
            </div>
            <div class="input-group">
              <span class="input-group-addon">
                <input type="checkbox" id="enabledSensors" checked>
              </span>
              <input type="text" class="form-control" value="Sensors" readonly="readonly">
            </div>



            <div id="VideoTopics">
              <h3 class="page-header">Video Configuration</h3>

              <div class="input-group">
                <span class="input-group-addon">Video Quality</span>
                <input type="text" class="form-control" placeholder="Value from 0-100" id="videoQuality">
              </div>
              <br/>
              <br />
              <div id="videotopics">
              </div>
              <button type="button" class="btn btn-default btn-md" style="width:100%;" id="addVideoTopic">
                <span class="glyphicon glyphicon-plus"></span> Add More Topics
              </button>
            </div>

            <div id="SensorTopics">
              <h3 class="page-header">Sensor Configuration</h3>

              <div id="sensortopics">
              </div>
              <button type="button" class="btn btn-default btn-md" style="width:100%;" id="addSensorTopic">
                <span class="glyphicon glyphicon-plus"></span> Add More Topics
              </button>
            </div>

          </div>
        </div>
        

        <button type="button" class="btn btn-success btn-lg" style="width:100%;" id="connectROS">
          <span class="glyphicon glyphicon-off"></span> Connect
        </button>


        <!-- EXPORT MODAL FOR CONNECTION -->
        <div class="modal fade" id="connectionExportModal">
          <div class="modal-dialog">
            <div class="modal-content">
              <div class="modal-header">
                <button type="button" class="close" data-dismiss="modal" aria-hidden="true">&times;</button>
                <h3 class="modal-title">Export Connection Configuration</h3>
              </div>
              <div class="modal-body">
                <p>Add the following code as a new addition to the <i>connectionConfig</i> variable in config.js located in the '/js/' directory (make sure to add commas between entries as need be):</p>
                <pre id="connectionExportPre" ></pre>
              </div>
              <div class="modal-footer">
                <button type="button" class="btn btn-default" data-dismiss="modal">Close</button>
              </div>
            </div>
          </div>
        </div>

        <!-- EXPORT MODAL FOR SUBSCRIPTIONS -->
        <div class="modal fade" id="subscriptionExportModal">
          <div class="modal-dialog">
            <div class="modal-content">
              <div class="modal-header">
                <button type="button" class="close" data-dismiss="modal" aria-hidden="true">&times;</button>
                <h3 class="modal-title">Export Subscription Configuration</h3>
              </div>
              <div class="modal-body">
                <p>Add the following code as a new addition to the <i>subscriptionConfig</i> variable in config.js located in the '/js/' directory (make sure to add commas between entries as need be):</p>
                <pre id="subscriptionExportPre" ></pre>
              </div>
              <div class="modal-footer">
                <button type="button" class="btn btn-default" data-dismiss="modal">Close</button>
              </div>
            </div>
          </div>
        </div>

      </div>



      <!-- DASHBOARD SCREEN -->
      <div id="dashboardcontainer" style="display:none;">
          <h1 class="page-header">Dashboard</h1>


          <div class="panel-group" id="divLog">
            <div class="panel panel-default">
              <div class="panel-heading">
                <h4 class="panel-title">
                    Message Log
                </h4>
              </div>
              <div id="collapseLog" class="panel-collapse collapse in">
                <div class="panel-body">
                  <div style="height:100px;display:block;overflow-y:auto;" id="scrollLog">
                    <table class="table table-striped" style="overflow: scroll;" >
                      <thead>
                         <tr>
                           <th>From</th>
                           <th>Timestamp</th>
                           <th>Message</th>
                         </tr>
                      </thead>
                      <tbody id="tbodyLog">
                      </tbody>
                    </table>
                  </div>
                </div>
              </div>
            </div>
          </div>


          <div class="panel-group" id="divVideo">
            <div class="panel panel-default">

              <div class="panel-heading">
                <h4 class="panel-title">
                  <a data-toggle="collapse" data-parent="#divVideo" href="#collapseVideo">
                    Video
                  </a>
                </h4>
              </div>

              <div id="collapseVideo" class="panel-collapse collapse out">
                <div class="panel-body">
                  <center>
                     <div id="mjpegVideoScreen">
                     </div>
                  </center>
                </div>
              </div>
            </div>
          </div>



          <div class="panel-group" id="divTeleop">
            <div class="panel panel-default">

              <div class="panel-heading">
                <h4 class="panel-title">
                  <a data-toggle="collapse" data-parent="#divTeleop" href="#collapseTeleop">
                    Teleoperation
                  </a>
                </h4>
              </div>

              <div id="collapseTeleop" class="panel-collapse collapse out">
                <div class="panel-body">
                  <center>
                     <div id="teleopSpeedLabel"></div>
                     <div id="teleopSpeedSlider"></div>
                     <br/>
                     <br/>
                     <div>
                        <button type="button" class="btn btn-default btn-xlarge" id="teleopLeftKey">
                          <span class="glyphicon glyphicon-arrow-left"></span>
                        </button>
                        <button type="button" class="btn btn-default btn-xlarge" id="teleopUpKey">
                          <span class="glyphicon glyphicon-arrow-up"></span>
                        </button>
                        <button type="button" class="btn btn-default btn-xlarge" id="teleopRightKey">
                          <span class="glyphicon glyphicon-arrow-right"></span>
                        </button>
                        <br/>
                        <!-- <button type="button" class="btn btn-default btn-xlarge" onclick="simulateKeyPress('')">
                          <span class="glyphicon glyphicon-stop"></span>
                        </button> -->
                        <button type="button" class="btn btn-default btn-xlarge" id="teleopTurnLeftKey">
                          <span class="glyphicon glyphicon-repeat icon-rotate icon-flipped"></span>
                        </button>
                        <button type="button" class="btn btn-default btn-xlarge" id="teleopDownKey">
                          <span class="glyphicon glyphicon-arrow-down"></span>
                        </button>
                        <button type="button" class="btn btn-default btn-xlarge" id="teleopTurnRightKey">
                          <span class="glyphicon glyphicon-repeat"></span>
                        </button>
                     </div>
                  </center>
                </div>
              </div>
            </div>
          </div>


          <div class="panel-group" id="divNav">
            <div class="panel panel-default">

              <div class="panel-heading">
                <h4 class="panel-title">
                  <a data-toggle="collapse" data-parent="#divNav" href="#collapseNav">
                    Navigation
                  </a>
                </h4>
              </div>

              <div id="collapseNav" class="panel-collapse collapse out">
                <div class="panel-body">
                  <center>
                     <div id="navWindow">
                     </div>
                     <button type="button" class="btn btn-default btn-xlarge" id="navLeftKey" onclick="navAction('left')">
                        <span class="glyphicon glyphicon-arrow-left"></span>
                     </button>
                     <button type="button" class="btn btn-default btn-xlarge" id="navDownKey" onclick="navAction('down')">
                        <span class="glyphicon glyphicon-arrow-down"></span>
                     </button>
                     <button type="button" class="btn btn-default btn-xlarge" id="navUpKey" onclick="navAction('up')">
                        <span class="glyphicon glyphicon-arrow-up"></span>
                     </button>
                     <button type="button" class="btn btn-default btn-xlarge" id="navRightKey" onclick="navAction('right')">
                        <span class="glyphicon glyphicon-arrow-right"></span>
                     </button>
                     <button type="button" class="btn btn-default btn-xlarge" id="navZoomOut" onclick="navAction('zoomout')">
                         <span class="glyphicon glyphicon-zoom-out"></span>
                     </button>
                     <button type="button" class="btn btn-default btn-xlarge" id="navZoomIn" onclick="navAction('zoomin')">
                         <span class="glyphicon glyphicon-zoom-in"></span>
                     </button>
                  </center>
                </div>
              </div>
            </div>
          </div>



          <div class="panel-group" id="divSensors">
            <div class="panel panel-default">

              <div class="panel-heading">
                <h4 class="panel-title">
                  <a data-toggle="collapse" data-parent="#divSensors" href="#collapseSensors">
                    Sensors
                  </a>
                </h4>
              </div>

              <div id="collapseSensors" class="panel-collapse collapse out">
                <div class="panel-body">
                  <div class="table-responsive">
                   <table class="table table-striped" style="overflow: scroll;" >

                     <thead>
                       <tr>
                         <th>Topic</th>
                         <th class='hidden-xs hidden-sm'>Type</th>
                         <th>Updated</th>
                         <th>Value</th>
                       </tr>
                     </thead>

                     <!-- tbodyTableSubscriptions is filled out with initializeSubscribers() -->
                     <tbody id="tbodyTableSubscriptions">
                     </tbody>

                   </table>
                 </div>
                </div>
              </div>
            </div>
          </div>
        </div>

    </div>
    

    <script>

      
      //This function is called when the "CONNECT" button is pressed or when url parameters are loaded in
      //It sets up the proper variables, then connects to ROSBRIDGE
      function connectToROS()
      {
        //Get connection info
        hostInfo = createConnectionConfig($('#inputHost').val(), $('#inputRosbridgePort').val(), $('#inputMjpegServerPort').val());
        rosBridgeAddress = "ws://" + hostInfo.Host + ":" + hostInfo.RosbridgePort;
        rosMjpegVideoAddress = hostInfo.Host;
        rosMjpegVideoPort = hostInfo.MjpegServerPort;

        //Get enabled widgets
        enabledVideo = $('#enabledVideo').prop('checked');
        enabledTeleop = $('#enabledTeleop').prop('checked');
        enabledNavigation = $('#enabledNavigation').prop('checked');
        enabledSensors = $('#enabledSensors').prop('checked');


        //Get subscription info
        //video quality
        if(enabledVideo)
        {
          rosMjpegVideoQuality = $('#videoQuality').val();
          //video info
          rosMjpegVideoTopics = [];
          $('#videotopics').find('input').each(function () {
            if(this.value!="")
            {
              rosMjpegVideoTopics.push(this.value);
            }
          });
        }
        else
        {
          rosMjpegVideoQuality = 1;
          rosMjpegVideoTopics = [];
        }


        //sensor info
        if(enabledSensors)
        {
          tableSubscriptions = [];
          $('#sensortopics').children('div').each(function () {
            // rosMjpegVideoTopics.push(this.value);
            var tempTopicBuffer = [];
            $(this).find('input').each(function(){
              tempTopicBuffer.push(this.value);
            });
            if(tempTopicBuffer[0] != "")
            {
              var splitSubtopics = tempTopicBuffer[2].split(/[\s,]+/);
              tableSubscriptions.push(createSensorSubscriptionObject(tempTopicBuffer[0], tempTopicBuffer[1], splitSubtopics));
            }
          });
        }
        else
        {
          tableSubscriptions = [];
        }

        rosMjpegServerInfo = {
                               Host : rosMjpegVideoAddress,
                               Port : rosMjpegVideoPort,
                               Quality : rosMjpegVideoQuality, //0-100
                               Topics : rosMjpegVideoTopics,
                               Labels : rosMjpegVideoTopics
                              }





        $('#configurationcontainer').css({ "display": 'none'});
        $('#dashboardcontainer').css({ "display": ''});

        //Enable widgets that have been selected
        if(enabledVideo){ $('#divVideo').css({ "display": ''}); }
        else{ $('#divVideo').css({ "display": 'none'}); }

        if(enabledTeleop){ $('#divTeleop').css({ "display": ''}); }
        else{ $('#divTeleop').css({ "display": 'none'}); }

        if(enabledNavigation){ $('#divNav').css({ "display": ''}); }
        else{ $('#divNav').css({ "display": 'none'}); }

        if(enabledSensors){ $('#divSensors').css({ "display": ''}); }
        else{ $('#divSensors').css({ "display": 'none'}); }

        //initialize the components of the dashboard
        initializeROS();
        initializeSensorTable();
        initializeLogSubscriptions();
      }



      //Connect button has been pressed
      $('#connectROS').click( function() {
        connectToROS();
      });



      // Fires when the "Export" button next to the connection configuration is pressed
      // Shows the user the proper variable to put into the config.js file
      $('#connectionExport').click( function(){
          var exportStr = JSON.stringify(createConnectionConfig($('#inputHost').val(), $('#inputRosbridgePort').val(), $('#inputMjpegServerPort').val()), null, 2);
          exportStr = exportStr.replace(/\"([^(\")"]+)\":/g,"$1:");
          exportStr = '"your_config_name_here" : ' + exportStr;
          $('#connectionExportPre').text(exportStr);
          $('#connectionExportModal').modal('show');
      });

      // Fires when the "Export" button next to the subscription configuration is pressed
      // Shows the user the proper variable to put into the config.js file
      $('#subscriptionExport').click( function(){
          var exportStr = JSON.stringify(createConfigurationObject(), null, 2);
          exportStr = exportStr.replace(/\"([^(\")"]+)\":/g,"$1:");
          exportStr = '"your_config_name_here" : ' + exportStr;
          $('#subscriptionExportPre').text(exportStr);
          $('#subscriptionExportModal').modal('show');
      });

      // Used by the subscription export to create the proper variable info
      function createConfigurationObject()
      {
        var tempVideotopics = [];
        if($('#enabledVideo').prop('checked'))
        {
          $('#videotopics').find('input').each(function () {
              if(this.value!="")
              {
                tempVideotopics.push(this.value);
              }
            });
        }

        var tempSensortopics = [];
        if($('#enabledSensors').prop('checked'))
        {
          $('#sensortopics').children('div').each(function () {
            // rosMjpegVideoTopics.push(this.value);
            var tempTopicBuffer = [];
            $(this).find('input').each(function(){
              tempTopicBuffer.push(this.value);
            });
            if(tempTopicBuffer[0] != "")
            {
              var splitSubtopics = tempTopicBuffer[2].split(/[\s,]+/);
              tempSensortopics.push(createSensorSubscriptionObject(tempTopicBuffer[0], tempTopicBuffer[1], splitSubtopics));
            }
          });
        }



        var tempConfigObject = {
              Enabled: {
                          Video:   $('#enabledVideo').prop('checked'),
                          Teleop:  $('#enabledTeleop').prop('checked'),
                          Nav:     $('#enabledNavigation').prop('checked'),
                          Sensors: $('#enabledSensors').prop('checked')
                        },
              VideoQuality: $('#videoQuality').val(),
              VideoTopics: tempVideotopics,
              SensorTopics: tempSensortopics
                          };



        return tempConfigObject;
      }


        // Adds an entry to the log window at the top of the screen and scrolls down to the bottom
        function logToScreen(fromName, logInput)
        {
            var logMessage = logInput;
            if(logInput !== null && typeof logInput === 'object')
            {
              logMessage = JSON.stringify(logInput, null, 2);
            }
            var tbodyLog = document.getElementById('tbodyLog');
            var trLog = document.createElement('tr');

            var tdFrom = document.createElement('td');
            tdFrom.innerHTML = fromName;

            var tdUpdated = document.createElement('td');
            tdUpdated.innerHTML = formatNowTimestamp();

            var tdValue = document.createElement('td');
            tdValue.innerHTML = logMessage;

            trLog.appendChild(tdFrom);
            trLog.appendChild(tdUpdated);
            trLog.appendChild(tdValue);
            tbodyLog.appendChild(trLog);

            //scroll to bottom of div
            $('#scrollLog').scrollTop($('#scrollLog')[0].scrollHeight);
            // $("#scrollLog").animate({ scrollTop: $('#scrollLog')[0].scrollHeight}, 1000);
        }


      // Adds a field to the list of video topics on the configuration window
      // topicname is used as the value of the input
      function addVideoTopic(topicname)
      {
        $("#videotopics").append('<div class="input-group"><span class="input-group-addon">Topic Name</span><input type="text" class="form-control" placeholder="/nav_kinect/rgb/image_raw" value="' + topicname + '"></div><br/>');
      }

      // Adds the appropriate fields to the list of sensor subscriptions on the configuration window
      // the input variables are used as the initial values of the inputs
      function addSensorTopic(topicname, messagetype, topicrestrictions)
      {
        var topicrestrictionsString = '';
        for (var i = 0; i < topicrestrictions.length; i++) {
            topicrestrictionsString = topicrestrictionsString + topicrestrictions[i];
            if(i != topicrestrictions.length-1)
            {
              topicrestrictionsString = topicrestrictionsString + ', ';
            }
            //Do something
        }
        $("#sensortopics").append('<div><div class="input-group"><span class="input-group-addon">Topic Name</span><input type="text" class="form-control" placeholder="/odom" value="' + topicname + '"></div><div class="input-group"><span class="input-group-addon">Message Type</span><input type="text" class="form-control" placeholder="nav_msgs/Odometry" value="' + messagetype + '"></div><div class="input-group"><span class="input-group-addon">SubTopics</span><input type="text" class="form-control" placeholder="twist/twist/linear, twist/twist/angular" value="' + topicrestrictionsString + '"></div></div><br/>');
      }

      //Clears the list of video fields on the configuration screen
      function clearVideoTopics()
      {
        $("#videotopics").empty();
      }

      //Clears the list of sensor fields on the configuration screen
      function clearSensorTopics()
      {
        $("#sensortopics").empty();
      }

      // Fires when the "Add" button is pressed for the video topics on the configuration screen
      $('#addVideoTopic').click( function() {
        addVideoTopic('');
      });

      // Fires when the "Add" button is pressed for the sensor topics on the configuration screen
      $('#addSensorTopic').click( function() {
        addSensorTopic('','',[]);
      });

      
      // Fires when a new subscription default is chosen on the configuration screen
      $('#subscriptionSelect').change( function() {    
        clearVideoTopics();
        clearSensorTopics();
        if($(this).val()=="CUSTOM")
        {
          // The user wants to put in their own values, reset all the fields
          addSensorTopic('','',[]);
          addVideoTopic('');
          $('#enabledVideo').prop('checked', true);
          $('#enabledTeleop').prop('checked', true);
          $('#enabledNavigation').prop('checked', true);
          $('#enabledSensors').prop('checked', true);
          $('#videoQuality').val('');
          $('#VideoTopics').css({ "display": ''});
          $('#SensorTopics').css({ "display": ''});
        }
        else
        {
          //initialize content
          var subscrConfig = subscriptionConfig[$(this).val()];
          var subscrVideo = subscrConfig.VideoTopics;
          var subscrSensor = subscrConfig.SensorTopics;

          $('#videoQuality').val(subscrConfig.VideoQuality);
          $('#enabledVideo').prop('checked', subscrConfig.Enabled.Video);
          $('#enabledTeleop').prop('checked', subscrConfig.Enabled.Teleop);
          $('#enabledNavigation').prop('checked', subscrConfig.Enabled.Nav);
          $('#enabledSensors').prop('checked', subscrConfig.Enabled.Sensors);

          if(subscrConfig.Enabled.Video)
          {
            $('#VideoTopics').css({ "display": ''});
          }
          else
          {
            $('#VideoTopics').css({ "display": 'none'});
          }

          if(subscrConfig.Enabled.Nav)
          {
            $('#NavTopics').css({ "display": ''});
          }
          else
          {
            $('#NavTopics').css({ "display": 'none'});
          }

          if(subscrConfig.Enabled.Sensors)
          {
            $('#SensorTopics').css({ "display": ''});
          }
          else
          {
            $('#SensorTopics').css({ "display": 'none'});
          }


          for(var i=0; i<subscrVideo.length; i++)
          {
            addVideoTopic(subscrVideo[i]);
          }

          for(var i=0; i<subscrSensor.length; i++)
          {
            addSensorTopic(subscrSensor[i].TopicName, subscrSensor[i].MessageType, subscrSensor[i].SubTopicNames);
          }

        }

      });

      // Fires when a new connection default is chosen on the configuration screen
       $('#connectionSelect').change( function() {    
        if($(this).val()=="CUSTOM")
        {
          // The user wants to put in their own values, reset all the fields
          hostInfo = createConnectionConfig("","","");
        }
        else
        {
          hostInfo = connectionConfig[$(this).val()];
        }

        inputHost.value=hostInfo.Host;
        inputRosbridgePort.value=hostInfo.RosbridgePort;
        inputMjpegServerPort.value=hostInfo.MjpegServerPort;

      });

      $("#enabledVideo").change(function() {
          if(this.checked)
          {
            $('#VideoTopics').css({ "display": ''});
          }
          else
          {
            $('#VideoTopics').css({ "display": 'none'});
          }
      });

      $("#enabledNavigation").change(function() {
          if(this.checked)
          {
            $('#NavTopics').css({ "display": ''});
          }
          else
          {
            $('#NavTopics').css({ "display": 'none'});
          }
      });

      $("#enabledSensors").change(function() {
          if(this.checked)
          {
            $('#SensorTopics').css({ "display": ''});
          }
          else
          {
            $('#SensorTopics').css({ "display": 'none'});
          }
      });

      //When minimizing accordion for sensors, delete the subscribers so you're not wasting bandwith
      $('#collapseSensors').on('hidden.bs.collapse', function (e) {
          deleteSensorSubscribers();
      })
      //Since the subscribers were deleted upon collapsing accordion, recreate them when expanding
      $('#collapseSensors').on('shown.bs.collapse', function (e) {
          createSensorSubscribers();
      })

      //When minimizing accordion for videos, clear the screen so you're not wasting bandwith
      $('#collapseVideo').on('hidden.bs.collapse', function (e) {
           deleteVideoSubscribers();
      })
      //Since the subscribers were deleted upon collapsing accordion, recreate them when expanding
      $('#collapseVideo').on('shown.bs.collapse', function (e) {
          createVideoSubscribers();
      })


      //When minimizing accordion for navigation, clear the screen so you're not wasting bandwith
      $('#collapseNav').on('hidden.bs.collapse', function (e) {
           deleteNavSubscribers();
      })
      //Since the subscribers were deleted upon collapsing accordion, recreate them when expanding
      $('#collapseNav').on('shown.bs.collapse', function (e) {
          createNavSubscribers();
      })

      //When minimizing accordion for navigation, clear the screen so you're not wasting bandwith
      $('#collapseTeleop').on('hidden.bs.collapse', function (e) {
           deleteTeleopSubscribers();
      })
      //Since the subscribers were deleted upon collapsing accordion, recreate them when expanding
      $('#collapseTeleop').on('shown.bs.collapse', function (e) {
          createTeleopSubscribers();
      })


    </script>

  </body>
</html>
