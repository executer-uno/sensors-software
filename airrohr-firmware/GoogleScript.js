// Created by Sujay S. Phadke, 2017
// All Rights Reserved.
// Github: https://github.com/electronicsguy
//
// Read/Write to Google Sheets using REST API.
// Can be used with ESP8266 & other embedded IoT devices.
// 
// Use this file with the ESP8266 library HTTPSRedirect
// 
// doGet() and doPost() need the spreadsheet ID. Cannot use "active spreadsheet" here since
// the device can operate without the spreadsheet even being open.
// http://stackoverflow.com/questions/4024271/rest-api-best-practices-where-to-put-parameters
// http://trevorfox.com/2015/03/rest-api-with-google-apps-script

// Similar API docs:
// https://gspread.readthedocs.org/en/latest/
// https://smartsheet-platform.github.io/api-docs/#versioning-and-changes
// http://search.cpan.org/~jmcnamara/Excel-Writer-XLSX/lib/Excel/Writer/XLSX.pm

// http://forum.espruino.com/conversations/269510/
// http://stackoverflow.com/questions/34691425/difference-between-getvalue-and-getdisplayvalue-on-google-app-script
// http://ramblings.mcpher.com/Home/excelquirks/gooscript/optimize

// Things to remember with getValue() object format:
// 1. Partial dates or times-only will be replaced with a full date + time, probably in the
//    year 1989. Like this: Sat Dec 30 1899 08:09:00 GMT-0500 (EST)
// 2. Dollar ($) currency symbol will be absent if cell contains currency.
//    This may be locale-dependent.
// 3. Scientific notation will be replaced by decimal numbers like this: 0.0000055

// Script examples
// https://developers.google.com/adwords/scripts/docs/examples/spreadsheetapp
                                  
var SS = SpreadsheetApp.openById('xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx'); // ESP32 PM25 googlespreadsheet
var sheetAct = SS.getSheetByName('Actual');
var raw      = SS.getSheetByName('RAW'); 
var str = "";


function test() {
  var str = doPost("arguments row");
}


function doPost(e) {

  var parsedData;
  var result = {};

  var nowStamp = Utilities.formatDate(new Date(), "Europe/Kiev", "yyyy-MM-dd hh:mm:ss a");
  
  var contents = e.postData.contents;
  
  //Update actual data on RAW sheet
  
  if(raw.getMaxRows()>106){
    raw.deleteRow(106);
  }
    
  raw.insertRows(2);
  SS.getRange("RAW!A2").setValue(nowStamp);
  SS.getRange("RAW!B2").setValue(contents);
    
  
  try { 
    parsedData = JSON.parse(contents);
  } 
  catch(f){
    return ContentService.createTextOutput("Error in parsing request body: " + f.message);
  }
   
  if (parsedData !== undefined){
    // Common items first
    // data format: 0 = display value(literal), 1 = object value
    var flag = parsedData.format;
    
    if (flag === undefined){
      flag = 0;
    }
    
    switch (parsedData.command) {
      case "appendRow":
        
        var tmp = SS.getSheetByName(parsedData.sheet_name);
                 
        var now = Utilities.formatDate(new Date(), "Europe/Kiev", "yyyy-MM-dd hh:mm:ss a");
        
        // Update actual data on sheetAct
        sheetAct.getRange('Timestamp').setValue(now);
        sheetAct.getRange('Act_Count').setValue(parsedData.values.count_sends);
        sheetAct.getRange('StationID').setValue(parsedData.values.espid);
        sheetAct.getRange('SWversion').setValue(parsedData.values.software_version);

          
        if(tmp.getMaxRows()>100006){
          tmp.deleteRow(100006);
        }
        tmp.insertRows(5);
          
        //Put data to table
        tmp.getRange(5, 1).setValue(now);
        //PMS3007
        tmp.getRange(5, 3).setValue(parsedData.values.sensordatavalues.PMS_P1);
        tmp.getRange(5, 4).setValue(parsedData.values.sensordatavalues.PMS_P2);
        //SDS011
        tmp.getRange(5, 5).setValue(parsedData.values.sensordatavalues.SDS_P1);
        tmp.getRange(5, 6).setValue(parsedData.values.sensordatavalues.SDS_P2);
        //BME
        tmp.getRange(5, 7).setValue(parsedData.values.sensordatavalues.BME280_pressure);
        tmp.getRange(5, 8).setValue(parsedData.values.sensordatavalues.BME280_temperature);
        tmp.getRange(5, 9).setValue(parsedData.values.sensordatavalues.BME280_humidity);
        //GPS
        tmp.getRange(5, 10).setValue(parsedData.values.sensordatavalues.GPS_lat);
        tmp.getRange(5, 11).setValue(parsedData.values.sensordatavalues.GPS_lon);
        tmp.getRange(5, 12).setValue(parsedData.values.sensordatavalues.GPS_height);          
        
        //Heap
        tmp.getRange(5, 13).setValue(parsedData.values.sensordatavalues.Heap_Siz);
        tmp.getRange(5, 14).setValue(parsedData.values.sensordatavalues.Heap_Free);
        tmp.getRange(5, 15).setValue(parsedData.values.sensordatavalues.Heap_MinFree);
        tmp.getRange(5, 16).setValue(parsedData.values.sensordatavalues.Heap_MaxAlloc);

        tmp.getRange(5, 17).setValue(parsedData.values.count_sends);

        
        sheetAct.getRange('Act_PM025_Avg' ).setValue(parsedData.values.sensordatavalues.PMS_P1);
        sheetAct.getRange('Act_PM100_Avg' ).setValue(parsedData.values.sensordatavalues.PMS_P2);
        sheetAct.getRange('Act_SD025_Avg' ).setValue(parsedData.values.sensordatavalues.SDS_P1);
        sheetAct.getRange('Act_SD100_Avg' ).setValue(parsedData.values.sensordatavalues.SDS_P2);
        
        str = "Success";
        SpreadsheetApp.flush();          
          
    }
    
    return ContentService.createTextOutput(str);
  }
  
  else{
    return ContentService.createTextOutput("Error! Request body empty or in incorrect format.");
  }
  
  
}


function doGet(e){
  
  var val = e.parameter.value;
  var cal = e.parameter.cal;
  var read = e.parameter.read;
  
  if (cal !== undefined){
    return ContentService.createTextOutput(GetEventsOneWeek());
  }
  
  if (read !== undefined){
    var now = Utilities.formatDate(new Date(), "Europe/Kiev", "yyyy-MM-dd hh:mm:ss a").slice(11,30);
    sheetPar.getRange('readTStamp').setValue(now);
    var count = (sheetPar.getRange('readCount').getValue()) + 1;
    sheetPar.getRange('readCount').setValue(count);
    
    //var retparams =         ContentService.createTextOutput(sheetPar.getRange('AVG_CYCLES' ).getValue());
    //retparams = retparams + ContentService.createTextOutput(sheetPar.getRange('MEAS_PERIOD').getValue());
    
    return ContentService.createTextOutput(sheetPar.getRange(read).getValue());
  }
  
  if (e.parameter.value === undefined)
    return ContentService.createTextOutput("No value passed as argument to script Url.");
    
  var range = sheet.getRange('A1');
  var retval = range.setValue(val).getValue();
  var now = Utilities.formatDate(new Date(), "EST", "yyyy-MM-dd'T'hh:mm a'Z'").slice(11,19);
  sheetDat.getRange('B1').setValue(now);
  sheetDat.getRange('C1').setValue('0');
  
  if (retval == e.parameter.value)
    return ContentService.createTextOutput("Successfully wrote: " + e.parameter.value + "\ninto spreadsheet.");
  else
    return ContentService.createTextOutput("Unable to write into spreadsheet.\nCheck authentication and make sure the cursor is not on cell 'A1'." + retval + ' ' + e.parameter.value);
}

