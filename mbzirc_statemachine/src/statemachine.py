import rospy
from smach import State, StateMachine

from time import Sleep

class One(State):
        def __init__(self)
        State.__init__(self,outcomes=['success'])

        def execute (self, userdata):
            print 'One'
            sleep (1)
            return 'Success'

class Two(State):
        def __init__(self)
        State.__init__(self,outcomes=['success'])

        def execute (self, userdata):
            print 'Two'
            sleep (1)
            return 'Success'



if __name__=='__main__':
    sm=StateMachine(outcomes=['success'])
    with sm:
        StateMachine.add('ONE',One(),transitions={'success':'TWO'})
        StateMachine.add('TWO',Two(),transitions={'success':'ONE'})

    sm.execute()