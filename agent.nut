const FEED_ID = "YOURS";
const API_KEY = "YOURS";
lastFeedCSV <- "";
lastFeedJSON <- {};

function send_xively(body) {         //take in csv value
    local xively_url = "https://api.xively.com/v2/feeds/" + FEED_ID + ".csv";       //setup url for csv
    server.log(xively_url);
    server.log(body);       //pring body for testing
    local req = http.put(xively_url, {"X-ApiKey":API_KEY, "Content-Type":"text/csv", "User-Agent":"Xively-Imp-Lib/1.0"}, body);     //add headers
    local res = req.sendsync();         //send request
    if(res.statuscode != 200) {
        server.log("error sending message: "+res.body);
    }
    else {
        device.send("status", (res.statuscode + " OK"));      //sends status to uart. this can be removed if not desired
    }
}
  
device.on("data", function(feedCSV) {       //take csv body in from device
    server.log("received data: " + feedCSV);
     
    //send preformatted multi-ds csv
    lastFeedCSV = feedCSV;
    local items = split(feedCSV, "\n");
    foreach(item in items) {
        local pairs = split(item, ",");
        lastFeedJSON.rawset(pairs[0],pairs[1]);
    }
    send_xively(feedCSV);         //send to function to call xively
});

device.onconnect(function() {
    //server.log("sending configuration values");
    //device.send("setMode", "cool");    
    //device.send("changeSetpoint", "78.0");
});

http.onrequest(function(request, response) {
   //get JSON body
   if( request.method == "POST" ) {
       //new JSON request to set values
       local t = http.urldecode(request.body);
       server.log("request.body: " + t.data);
       local requestJSON = http.jsondecode(t.data);
       if( "setpoint" in requestJSON ) {
           device.send("changeSetpoint", requestJSON.setpoint);
       }
       if( "deadband" in requestJSON ) {
           device.send("changeDeadband", requestJSON.deadband);
       }
       if( "mode" in requestJSON ) {
           device.send("setMode", requestJSON.mode);
       }
       response.send(200, "OK");
       return;
   }
   //else, just report the latest values
   response.header("Content-Type", "text/html");
   local jsonString = http.jsonencode(lastFeedJSON);
   local html = "<html><head><title>MyImp</title></head>"
        + "<body><form method=POST action='"+http.agenturl()+"'>"
        + "<textarea name=data rows=4 cols=20>{ \"setpoint\" : \"78\", \"mode\" : \"cool\" }</textarea><br/>"
        + "<button name=send type=submit>send</button>"
        + "</form>"
        + "<p>Current state is: "+jsonString+"</p>"
        + "</body>";
   response.send(200, html);
});
