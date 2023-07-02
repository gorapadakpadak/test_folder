# Vision 2022


2022-alphacar에서 Vision Team은 다음 Task 를 진행합니다
* Cone Box 인식
* 배달 표지판 인식
* 신호등 인식


Vision Team 의 node는 Slam Launch 파일로 실행됩니다.

slam_cone.launch
```md
<node pkg="vision" name="ConeBoxNode" type="ConeBoxNode.py" output="screen"/>
```


slam_kcity_pre.launch
```md
  <node pkg="vision" name="TrafficLightNode" type="traffic_light_publisher.py" output="screen"/>
```

slam_fmtc.launch
```md
  <node pkg="vision" name="TrafficLightNode" type="traffic_light_publisher.py" output="screen"/>
  <node pkg="vision" name="DeliverySignNode" type="DeliverySignNode.py" output="screen"/>
```


slam_kcity_final.launch
```md
<node pkg="vision" name="TrafficLightNode" type="traffic_light_publisher.py" output="screen"/>
<node pkg="vision" name="DeliverySignNode" type="DeliverySignNode.py" output="screen"/>
```


## Base
DeliverySignNode / TrafficLightNode / ConeBoxNode 모두 Yolo v5를 base로 사용합니다.

base 모델 구조는 다음과 같습니다.
``` py
def publisher()
    1. ros node initialization
    2. load yolo v5 model (+ additional model)
    3. camera initialization
    while not rospy.is_shutdown():
        4. read image
        5. set exposure
        6. image preprocessing
        7. Detect object using yolo v5 
        8. Post processing (+ additional inference e.g. color prediction in ConeBoxNOde)
        9. Visualize and publish

```
아래의 3개의 node 전부 위의 구조를 따르며, 각 노드마다 8. post processing에서 약간의 차이가 존재합니다.


## DeliverySignNode
배달 표지판은 yolo5를 사용하여 검출합니다.
해당 모델의 weigth는 아래에 있습니다.
* DeliverySignNode/DeliverySignNode/delivery_20210901_yolov5s_640.pt
* DeliverySignNode/delivery_20210907_resnet007.pt

이 노드는 표지판 Detection + Classification , 2-stage로 구쉉되어 있습니다. 

*_yolov5s_640.pt 은 Delivery Sign Detection model 을 위한 weight입니다.

이 노드가 탐지해야 하는 배달 표지판은 다음과 같습니다.
![deliverySign](./asset/DeliverySign.png)

yolov5 를 사용하여 배달 표지판을 검출하면, 표지판 classifier( *_resnet007.pt weight 사용) 으로 분류합니다.

Resnet 기반의 Classifier 를 사용하였으며, (A1, A2, A3, B1, B2, B3, None) class로 분류합니다.

분류 과정에서 각 표지판의 id(label)은 다음과 같습니다.  
| class | id |  
|---| ---|
|A1 | 0|
|A2|1|
|A3|2|
|B1|3|
|B2|4|
|B3|5|

또 배달 표지판의 경우 크기가 일정하기 때문에, 검출된 box 크기를 사용하여 차량과 표지판 까지의 대략적인 거리를 slam 팀에 전달합니다.

distance 측정은 DeliverySignNode.Smooth.process()에서 이뤄지며, distance, x/y position , width.height 를 delivery_sign : np.ndarray 에 저장하여 전달합니다.
(Smooth.process() 의 return 확인)     


## TrafficLightNode
신호등은 yolo5를 사용하여 검출합니다.
이 노드는 단순하게 Traffic Light 를 Detection 하기 보다는, 현재 차량이 어느 신호를 받는지 판단합니다.
이 내용은 아래의 TrafficLightNode.Smooth.process() function 에서 자세하게 다루겠습니다.

이 노드에서 사용하는 모델의 weigth는 아래에 있습니다.
* TrafficLightNode/traffic_light_20210824_yolov5m_640.pt"

*_yolov5s_640.pt 은 Traffic Light Detection model 을 위한 weight입니다.

yolo 모델이 Detection 한 신호등의 id(label) 은 다음과 같습니다.
| class | id |  
|---| ---|
|No Traffic Light | -1|
|Red|0|
|Red Left|1|
|Green|2|
|Green Left|3|

한 이미지에서 여러 개의 신호등이 Detection 되는 경우, 판단에 어려움을 겪을수도 있습니다. (e.g. 가까운 신호등은 Green, 다음 신호등이 Red)

이를 처리하기 위해, TrafficLightNode.Smooth.process() func을 사용합니다.
process() 함수에서는 이미지 side에서 검출된 신호등 정보를 제외시키면서, 이전 frame 에서 판단한 신호정보를 누적하여 현재 신호를 판단합니다.   

e.g.  
TrafficLightNode.Smooth.process() :  
``` py
self.mov_prob = 0.875 * self.mov_prob + 0.125 * prob
```



## ConeBoxNode
이 노드에서 8. Post Processing 과정은 다음과 같습니다.
* yolo를 사용하여 Cone을 검출
* 검출된 Cone을 Bounding Box로 Crop
* Cone 모양으로 masking 하여 색상정보를 추출하여 색상 판단

이 노드에서 사용하는 yolo 모델의 weigth는 아래에 있습니다.
* ConeBoxNode/cone_20210828_yolov5s_640.pt



대회에서는 노란색 / 파란색 Cone을 구분하여야 합니다.
하지만 노란색 / 파란색이 구분된 Cone Dataset 의 부족으로 yolo end-to-end 방식보다는, Cone detection using yolo + classification 의 2-stage 방식을 선택하였습니다.

Cone detection 의 예시는 아래와 같습니다.

e.g. Detection 예시
![Cone_Detection_example](./asset/ConeDetection.png)

위의 예시에서 blue/yellow 를 구분하지 않고 Cone 을 Detection 한 것을 볼 수 있습니다.

위의 Detection 결과를 사용하여, Bounding Box모양으로 Crop한 후 masking하여 Cone을 분류합니다.

이 떄 사용하는 mask는 아래와 같습니다.
``` py
MASK = (
    np.array(
        [
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 2, 3, 2, 0, 0, 0],
            [0, 0, 2, 3, 3, 3, 2, 0, 0],
            [0, 1, 2, 3, 3, 3, 2, 1, 0],
        ],
        dtype=np.uint8,
    ) * 85
)
```




## Source
Vision 파트의 Code는 Yolov5 의 official repo 를 많이 참고하였습니다.

* https://github.com/ultralytics/yolov5/tree/master/utils

각 노드의 utils 또는 models folder 코드에 대해 궁금하시다면, 위의 repo 를 참고해주시길 바랍니다.