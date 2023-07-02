
(cl:in-package :asdf)

(defsystem "yolov7_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ObjectTracking2D" :depends-on ("_package_ObjectTracking2D"))
    (:file "_package_ObjectTracking2D" :depends-on ("_package"))
    (:file "ObjectTracking2D" :depends-on ("_package_ObjectTracking2D"))
    (:file "_package_ObjectTracking2D" :depends-on ("_package"))
    (:file "ObjectTracking2DArray" :depends-on ("_package_ObjectTracking2DArray"))
    (:file "_package_ObjectTracking2DArray" :depends-on ("_package"))
    (:file "ObjectTracking2DArray" :depends-on ("_package_ObjectTracking2DArray"))
    (:file "_package_ObjectTracking2DArray" :depends-on ("_package"))
    (:file "yolo_class" :depends-on ("_package_yolo_class"))
    (:file "_package_yolo_class" :depends-on ("_package"))
    (:file "yolo_class" :depends-on ("_package_yolo_class"))
    (:file "_package_yolo_class" :depends-on ("_package"))
  ))