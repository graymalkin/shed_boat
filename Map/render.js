/*
 * JavaScript file to describe how to render the webpage to phantomjs
 */

var page = require('webpage').create();
page.viewportSize = { width: 1337, height: 850};

page.open('./Map/index.html', window.setTimeout(function() {
    page.render('./Map/map.png');
    phantom.exit();
}, 3000));
