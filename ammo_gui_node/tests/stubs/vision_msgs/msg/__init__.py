from _bag import bag

ObjectHypothesis = bag('ObjectHypothesis', class_id='', score=0.0)
_Result = bag('ObjectHypothesisWithPose', hypothesis=ObjectHypothesis)
Detection2D = bag('Detection2D', results=list, bbox=None)
Detection2DArray = bag('Detection2DArray', detections=list, header=None)
