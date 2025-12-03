world { X:[0 0 .1] }

#thick radius floor and walls
floor (world){ shape:ssBox, Q:[0 0 -0.05], size:[4.1 4.1 .1 .04], color:[0.7686 0.6863 .6471], contact: 0 friction:10, logical:{table} }

block_right (world){ shape:ssBox, Q:[0 -2. 0.1], size:[4.1 .1 0.2 .04], logical:{static}, color:[0.6953 0.515625 .453125], contact: 1 }
block_back (world){ shape:ssBox, Q:[2. 0 0.1], size:[.1 4.1 0.2 .04], logical:{static}, color:[0.6953 0.515625 .453125], contact: 1 }
block_left (world){ shape:ssBox, Q:[0 2. 0.1], size:[4.1 .1 0.2 .04], logical:{static}, color:[0.6953 0.515625 .453125], contact: 1 }
block_front (world){ shape:ssBox, Q:[-2. 0 0.1], size:[.1 4.1 0.2 .04], logical:{static} , color:[0.6953 0.515625 .453125], contact: 1 }

#sub-goal1 (floor){ shape:ssBox, Q:<t(-0.7 -0.6 .0)>, size:[0.1 0.1 .1 .005], color:[0.3 .3 .3 0.9], contact:0, logical:{table} }

cam0(world){ Q:"t(0 0 10) d(180 1 0 0)" shape:camera width:300 height:300}
#camera_gl(world){ Q:"t(0 0 16) d(180 1 0 0)" shape:camera width:900 height:900}


#camera_gl(world){ Q:"t(0 -13 13) d(-135 1 0 0)" shape:camera width:900 height:900}


