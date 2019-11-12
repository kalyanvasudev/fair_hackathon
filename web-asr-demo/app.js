const express = require('express');
const util = require('util');
const app = express();
const port = 3000;
const fs = require('fs');

app.get('/',function(req,res) {
    res.sendFile('html/webspeechdemo.html', {'root': '.'});
});

app.get('/send-text', function (req, res) {
    console.log("Received:" + JSON.stringify(req.query));
    console.log('Raw query text was: ' + req.query.text);
    decodedText = decodeURI(req.query.text) + '\n';
    var d = new Date();
    lineToWrite = d.getTime() + '|' + decodedText + '\n';

    fs.writeFile('data/robot_commands_asr.txt', lineToWrite, (err) => {
        if (err) throw err;
        console.log('Successfully saved decoded text to file: ' + lineToWrite);
    });
    res.end();
})

app.listen(port, () => console.log(`Example app listening on port ${port}!`))