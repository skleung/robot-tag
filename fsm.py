import Queue
import time

class State:
  def __init__(self):
    self.name = ''
    self.transitions = {}

  # Takes a transition name and assigns it to a callback
  def add_transition(self, transition, callback):
    self.transitions[transition] = callback


class StateMachine:
  def __init__(self, robot, are_it=False):
    self.states = {}
    self.currentState = None
    self.robot = robot
    self.speed = 30
    self.are_it = are_it
    if self.are_it:
      self.speed = 60
    self.queue = Queue.Queue()
    # self.collisionQueue = Queue.Queue()

    walk_state = self.add_state("Walk")
    self.set_start("Walk")
    walk_state.add_transition("line left", self.turn_left)
    walk_state.add_transition("line both", self.move_up)
    walk_state.add_transition("line right", self.turn_right)
    walk_state.add_transition("robot left", self.respondLeft)
    walk_state.add_transition("robot right", self.respondRight)
    walk_state.add_transition("tagged", self.turnOff)
    walk_state.add_transition("got tagged", self.turnOn)
    # walk_state.add_transition("robot ahead", self.respondAhead)

  def set_light(self, on):
    if on:
      self.robot.set_led(0,4)
      self.robot.set_led(1,4)
    else:
      self.robot.set_led(0,1)
      self.robot.set_led(1,1)

  def turnOff(self):
    print "turning off"
    self.set_light(False)
    self.speed = 30
    self.move_up()
    self.are_it = False

  def turnOn(self):
    print "turning on"
    self.set_light(True)
    self.speed = 60
    self.move_up()
    self.are_it = True

  def add_state(self, name):
    a_state = State()
    a_state.name = name
    self.states[name] = a_state
    return a_state

  def set_start(self, name):
    self.currentState = name

  def clearQueue(self):
    collision_event = None
    while(not self.queue.empty()):
      temp = self.queue.get()
      if (temp[0] == "tagged" or temp[0] == "got tagged"):
        collision_event = temp
    self.queue.queue.clear()
    self.queue.put(collision_event)

  def clearCollisionQueue(self):
    self.collisionQueue.queue.clear()

  def respondAhead(self):
    if not self.are_it:
      self.move_down()

  def respondRight(self):
    if self.are_it:
      self.turn_right()
    else:
      self.turn_left()

  def respondLeft(self):
    if not self.are_it:
      self.turn_right()
    else:
      self.turn_left()

  # TODO: Refactor turning to use states to enter and break out
  def turn_right(self):
    self.move_right()
    time.sleep(0.05)
    self.clearQueue()
    self.move_up()

  def turn_left(self):
    self.move_left()
    time.sleep(0.05)
    self.clearQueue()
    self.move_up()

  def back_up(self):
    self.move_down()
    time.sleep(1)
    self.move_right()
    time.sleep(0.5)
    self.clearQueue()
    self.move_up()

  def move_up(self):
    self.robot.set_wheel(0,self.speed)
    self.robot.set_wheel(1,self.speed)

  def move_down(self):
    self.robot.set_wheel(0,-self.speed)
    self.robot.set_wheel(1,-self.speed)

  def move_left(self):
    self.robot.set_wheel(0,self.speed)
    self.robot.set_wheel(1,-self.speed)

  def move_right(self):
    self.robot.set_wheel(0,-self.speed)
    self.robot.set_wheel(1,self.speed)

  def run(self):
    self.move_up()
    while(True):
      state = self.states[self.currentState]

      transition, next_state = self.queue.get()
      if transition in state.transitions and next_state in self.states:
        state.transitions[transition]()
        self.currentState = next_state
        time.sleep(0.1)
