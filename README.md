# Tile_Airtag_Presence_Detector
Detect the presence of a specific Tile or similar BLE Tag

This is a presence detector based on Tile BLE Tags (but it could 
just as easily work with Airtags).

Taking D0 Low puts it into scanning mode.  Tags are sorted by decreasing
signal strength.  When D0 goes High, the tag with the strongest signal is stored.
Normally, at power-up, it scans for the stored tag.  If detected, flash the LED
and make D1 High.  After 30 seconds of no contact with the tag, D1 goes low.

I'm using this as a kind of valet mode for my DIY Electric vehicle.
If my tag is detected, it enables full speed & power.  If not, speed is limited to
10mph with unexciting acceleration.

I've written it to run on XIAO ESP32C6 Hardware - but it can easily be adapted to run on any other ESP32
