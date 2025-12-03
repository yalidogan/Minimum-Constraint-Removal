Include: <../base-walls-min-heatmap.g>
cam0(world){ Q:"t(0 0 10) d(180 1 0 0)" shape:camera width:300 height:300}
#########################################################################################################################
wall1 (world){ shape:ssBox, Q:[-1.1 1 0.2], size:[1.8 .1 0.4 .04] , color:[1 0 0], contact: 1 }

wall2 (world){ shape:ssBox, Q:[1.1 -1 0.2], size:[1.8 .1 0.4 .04] , color:[1 0 0], contact: 1 }

#########################################################################################################################
goal1 (floor){ shape:ssBox, Q:"t(-.8 1.5 .1)",  size:[.2 .4 .2 .02], color:[0 0 1 .5], contact:0, joint:rigid,  logical={ goal } }
goal2 (floor){ shape:ssBox, Q:"t(-1.2 1.5 .1)", size:[.2 .4 .2 .02], color:[0 0 1 .5], contact:0, joint:rigid,  logical={ goal } }
goal3 (floor){ shape:ssBox, Q:"t(-1.6 1.5 .1)", size:[.2 .4 .2 .02], color:[0 0 1 .5], contact:0, joint:rigid,  logical={ goal } }
#########################################################################################################################
egoJoint(world){Q:[0 0 0.2]  }
ego(egoJoint) {
    shape:ssCylinder, Q:[-1 0 0], size:[.4 .02 .02], color:[1 1 0]
}
##############################################################################################################################
obj1Joint(world){ Q:[.8 -1.5 0.1] }
obj1(obj1Joint) { 
    type:ssBox size:[.2 .4 .2 .02] color:[1 0 0],  logical={ object },
     joint:rigid, contact: 1 
}
obj1_cam(obj1){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}
##############################################################################################################################
obj2Joint(world){ Q:[1.2 -1.5 0.1] }
obj2(obj2Joint) { type:ssBox size:[.2 .4 .2 .02] color:[1. 0. 0],  logical={ object }, joint:rigid, contact: 1 }
obj2_cam(obj2){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

##############################################################################################################################
obj3Joint(world){ Q:[1.6 -1.5 0.1] }
obj3(obj3Joint) { type:ssBox size:[.2 .4 .2 .02] color:[1. 0. 0],  logical={ object }, joint:rigid, contact: 1 }
obj3_cam(obj3){ Q:"t(0 0 7) d(180 1 0 0)" shape:camera width:300 height:300}

##############################################################################################################################
