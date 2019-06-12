# rosplan_knowledge_base - unit tests

## build tests manually

build rosplan_knowledge_base tests:

        catkin build --no-deps rosplan_knowledge_base --make-args run_tests

build all catkin ws tests:

        catkin build --make-args run_tests

## run tests manually

Knowledge base tests:

        rostest rosplan_knowledge_base knowledge_base.test --text
