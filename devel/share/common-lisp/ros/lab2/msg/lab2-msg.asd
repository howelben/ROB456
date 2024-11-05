
(cl:in-package :asdf)

(defsystem "lab2-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "NavTargetAction" :depends-on ("_package_NavTargetAction"))
    (:file "_package_NavTargetAction" :depends-on ("_package"))
    (:file "NavTargetActionFeedback" :depends-on ("_package_NavTargetActionFeedback"))
    (:file "_package_NavTargetActionFeedback" :depends-on ("_package"))
    (:file "NavTargetActionGoal" :depends-on ("_package_NavTargetActionGoal"))
    (:file "_package_NavTargetActionGoal" :depends-on ("_package"))
    (:file "NavTargetActionResult" :depends-on ("_package_NavTargetActionResult"))
    (:file "_package_NavTargetActionResult" :depends-on ("_package"))
    (:file "NavTargetFeedback" :depends-on ("_package_NavTargetFeedback"))
    (:file "_package_NavTargetFeedback" :depends-on ("_package"))
    (:file "NavTargetGoal" :depends-on ("_package_NavTargetGoal"))
    (:file "_package_NavTargetGoal" :depends-on ("_package"))
    (:file "NavTargetResult" :depends-on ("_package_NavTargetResult"))
    (:file "_package_NavTargetResult" :depends-on ("_package"))
  ))