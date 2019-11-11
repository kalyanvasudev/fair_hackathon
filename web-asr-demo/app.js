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

    fs.writeFile('data/robot_commands_asr.txt', decodedText, (err) => {
        if (err) throw err;
        console.log('Successfully saved decoded text to file: ' + decodedText);
    });
    res.end();
})

app.listen(port, () => console.log(`Example app listening on port ${port}!`))