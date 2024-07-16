from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
import rospy

from skiros2_skill.core.skill import SkillBase, SelectorStar,Selector, Serial, State, SerialStar, ParallelFs,ParallelFf
from skiros2_common.core.primitive import PrimitiveBase

import skiros2_msgs.srv as srvs

import json



#################################################################################
# Wait
#################################################################################


class Wait(SkillDescription):
    """
    @brief Returns Running for a specific amount of time
    """

    def createDescription(self):
        # =======Params=========
        self.addParam("Duration", 0.0, ParamTypes.Required)


class wait(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Wait(), self.__class__.__name__)

    def onPreempt(self):
        return self.step("prempted")

    def onStart(self):
        self.first = 1
        return True

    def execute(self):
        if self.first == 1:
            self.first = 2
            self.last = rospy.Time.now()

        duration = rospy.Time.now() - self.last
        if self.params["Duration"].value == 0.0:
            return self.fail("The time is 0 sor I return a failure",-1)
        if duration.to_sec() > self.params["Duration"].value:
            return self.success("Done")
        return self.step("Waiting {}".format(self.params["Duration"].value)+" "+str(duration.to_sec()))


#################################################################################
# Set relation
#################################################################################

class WmSetRelation(SkillDescription):
    """
    @brief Set a relation on the world model
    """

    def createDescription(self):
        # =======Params=========
        self.addParam("Src", Element("sumo:Object"), ParamTypes.Required)
        self.addParam("Relation", str, ParamTypes.Required)
        self.addParam("Dst", Element("sumo:Object"), ParamTypes.Required)
        self.addParam("RelationState", True, ParamTypes.Required)


class wm_set_relation(PrimitiveBase):
    def createDescription(self):
        self.setDescription(WmSetRelation(), self.__class__.__name__)

    def execute(self):
        src = self.params["Src"].value
        relation = self.params["Relation"].value
        dst = self.params["Dst"].value
        if self.params["RelationState"].value:
            src.setRelation("-1", relation, dst.id)
            dst.setRelation(src.id, relation, "-1")
        else:
            rel = dst.getRelation(src.id, relation, "-1")
            if rel is not None:
                dst.removeRelation(rel)
            rel = src.getRelation("-1", relation, dst.id)
            if rel is not None:
                src.removeRelation(rel)
        self.params["Src"].value = src
        self.params["Dst"].value = dst
        return self.success("{} {}-{}-{}".format("Set" if self.params["RelationState"].value else "Unset", src.id, relation, dst.id))

#################################################################################
# Set relation
#################################################################################


class WmSetProperties(SkillDescription):
    """
    @brief Set some properties on an element
    """

    def createDescription(self):
        # =======Params=========
        self.addParam("Src", Element("sumo:Object"), ParamTypes.Required)
        self.addParam("Properties", dict, ParamTypes.Required)


class wm_set_properties(PrimitiveBase):
    def createDescription(self):
        self.setDescription(WmSetProperties(), self.__class__.__name__)

    def execute(self):
        src = self.params["Src"].value
        props = self.params["Properties"].value
        for k, v in props.items():
            src.setProperty(k, v)
        self.params["Src"].value = src
        return self.success("Setted properties to {}. {}".format(src.id, props))

#################################################################################
# WmMoveObject
#################################################################################


class WmMoveObject(SkillDescription):
    """
    @brief Move an Object from StartLocation to TargetLocation in the world model
    """

    def createDescription(self):
        # =======Params=========
        self.addParam("StartLocation", Element("sumo:Object"), ParamTypes.Inferred)
        self.addParam("TargetLocation", Element("sumo:Object"), ParamTypes.Optional)
        self.addParam("Object", Element("sumo:Object"), ParamTypes.Required)
        self.addParam("Relation", "skiros:contain", ParamTypes.Required)
        # =======PreConditions=========
        self.addPreCondition(self.getRelationCond("StartContainObj", "skiros:spatiallyRelated", "StartLocation", "Object", True))
        # =======PostConditions=========
        #self.addPostCondition(self.getRelationCond("TargetContainObj", "skiros:spatiallyRelated", "TargetLocation", "Object", True))


class wm_move_object(PrimitiveBase):
    """
    Instant primitive

    Set Target-Contain-Object on the world model
    """

    def createDescription(self):
        self.setDescription(WmMoveObject(), self.__class__.__name__)

    def onEnd(self):
        self.params["StartLocation"].unset()
        return True

    def execute(self):
        start = self.params["StartLocation"].value
        target = self.params["TargetLocation"].value if self.params["TargetLocation"].value.id else start
        objectt = self.params["Object"].value
        rel = objectt.getRelation(pred=self._wmi.get_sub_properties("skiros:spatiallyRelated"), obj="-1")

        start.setProperty("skiros:ContainerState", "Empty")
        target.setProperty("skiros:ContainerState", "Full")
        rel["src"] = target.id
        rel["type"] = self.params["Relation"].value
        self._wmi.update_element_properties(start)
        self._wmi.update_element_properties(target)
        self.params["Object"].value = objectt
        return self.success("{} moved from {} to {}.".format(objectt.id, start.id, target.id))

#################################################################################
# Counter
#################################################################################


class Counter(SkillDescription):
    """
    @brief      Returns Success after a number of tick
    """

    def createDescription(self):
        self.addParam("CountTarget", int, ParamTypes.Required)


class counter(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Counter(), self.__class__.__name__)

    def _print_count(self):
        return "{}/{}".format(self._counter, self.params["CountTarget"].value)

    def onPreempt(self):
        return self.step(self._print_count())

    def onStart(self):
        self._counter = 0
        return True

    def execute(self):
        if self._counter < self.params["CountTarget"].value:
            self._counter += 1
            return self.step(self._print_count())
        else:
            return self.success(self._print_count())



#################################################################################
# Any behavior tree execution - through json generated by for example owl-bt
#################################################################################


class AnySkill(SkillDescription):
    """
    @brief returns moving until "done"
    """
    def createDescription(self):
        # =======Params=========
        self.addParam("behaviorTree","owlbt-tree", ParamTypes.Required)

        
class any_skill(SkillBase):
    def createDescription(self):
        self.setDescription(AnySkill(), self.__class__.__name__)

    def expand(self,skill):
        f = open ("/home/ros/behaviorTree/"+self.params["behaviorTree"].value)

        children_skill = ""
        bt_json = json.load(f)
        bt_type = bt_json["type"]

        if bt_type =="ParallelFf" or bt_type == "SerialStar" or bt_type == "ParallelFs" or bt_type=="Selector":
                        child_processor = bt_type
                        children = bt_json["childNodes"]
                        children_skill += self.getChildren(children,child_processor)




        stringSkillBt ="skill("+children_skill+")"
        print(stringSkillBt)
        eval(stringSkillBt)

        
    def getChildren(self,children,processor="SerialStar"):
        i = 0
        child_skill = []
        child_skill_i = 0
        children_skill = ""
        while i < len(children):

            node_name = children[i]["type"]
            
            # define parameters of skill
            stringparam = "{ "
            if "properties" in children[i]:
                for param in children[i]["properties"]:
                    try:
                        param["value"] = float(param["value"])
                        quotes = ""
                        stringparam += '"'+param["name"] +'":'+quotes+ str(param["value"]) +""+quotes + ","

                    except ValueError:    
                        quotes = "\""

            stringparam = stringparam[:-1] # remove last comma
            stringparam += "}"

            if node_name =="ParallelFf" or node_name == "SerialStar" or node_name == "ParallelFs" or node_name=="Selector":
                children_skill +=self.getChildren(children=children[i]["childNodes"], processor=node_name)

            else :
                skill_type = self.getSkillsType(node_name)
                if (skill_type):
                    children_skill += 'self.skill("'+skill_type+'", "'+node_name+'",specify='+stringparam+')'
                else : 
                    log.error("[behaviortree error]", "the skill type is not known of %s"%node_name)

                    return ""
                child_skill_i = child_skill_i +1

            #else:
            #    print("node_name " +node_name+ " is not defined as a skill or a processor")
                          
            if i+1 == len(children):
                skill= " self.skill("+processor+"())("+children_skill+")"
                return skill
            else:
                children_skill += ","
            i=i+1



    def getSkillsType(self,skill_implementation):
        self._get_skills = rospy.ServiceProxy('/infraflex_robot/get_skills', srvs.ResourceGetDescriptions)
        self._skill_list = dict()

        msg = srvs.ResourceGetDescriptionsRequest()
        try:
            resp1 = self._get_skills(msg)
            res = resp1
        except rospy.ServiceException as e:
            log.error("[call]", "Service call failed: %s"%e)
            return
        if not res:
                log.error("[{}]".format(self.__class__.__name__), "Can t retrieve skills.")
        else:
            for c in res.list:
                if c.name == skill_implementation:
                    return c.type

            return None     


#################################################################################
# Instantiation of owl-bt behavior tree
#################################################################################

class InstantiateBehaviorTree(SkillDescription):
    """
    @brief updates owl-bt.json with the skiros skills
    """
    def createDescription(self):
        # =======Params=========
        self.addParam("behaviorTreeinit","owl-bt.json", ParamTypes.Required)

class instantiateBehaviorTree(PrimitiveBase):
    def createDescription(self):
        self.setDescription(InstantiateBehaviorTree(), self.__class__.__name__)


    def onPreempt(self):
        return self.success("Preempted")


    def execute(self):
       
        emptypath = "/home/ros/behaviorTree/empty-"+self.params["behaviorTreeinit"].value
        path = "/home/ros/behaviorTree/"+self.params["behaviorTreeinit"].value

        skirosSkills = self.getSkills()
        
        """
          "name": "wait",
          "icon": "hourglass-half",
          "description": "Wait for \"{{Duration}}\" seconds",
          "isComposite": false,
          "isPrimitive": true,
          "skirosname":"wait",
          "properties": [
            {
              "name": "Duration",
              "default": 0,
              "type": "number",
              "min": 0,
              "max": "10"
            }
          ]
        """
        with open (emptypath,"r") as jsonFile:
            data = json.load(jsonFile)
        
        for skirosSkill in skirosSkills:
            owlSkill = dict()
            owlSkill["name"] = skirosSkill.name
            owlSkill["icon"] = "hand"
            owlSkill["description"] = skirosSkill.name 
            owlSkill["isComposite"] = False
            owlSkill["isPrimitive"] = True
            owlSkill["skirosname"] = skirosSkill.type

            owlSkill["properties"]=[]
            i=0
            for jsonSkirosParam in skirosSkill.params:
                owlPropertie=dict()
                # TODO
                skirosParam = json.loads(jsonSkirosParam.param) 
                # example
                # "{"values": [{"relations": [], "last_update": 0.0, "id": "", "label": "", "type": "sumo:Agent", "properties": {}}], "specType": 2, "type": "skiros_wm::Element", "description": "", "key": "Robot"}"
                # "{"values": ["owl-bt.json"], "specType": 0, "type": "str", "description": "", "key": "behaviorTreeinit"}"
                owlPropertie["name"] = skirosParam["key"]
                owlPropertie["default"] = [] if skirosParam["values"]==[] else skirosParam["values"][0]
                owlPropertie["type"] = skirosParam["type"] # to be mapped to owl types
                if owlPropertie["type"] == "number":
                    owlPropertie["min"] = -1000
                    owlPropertie["max"] = 1000

                owlSkill["properties"].append(owlPropertie)

            data["nodes"].append(owlSkill)

        # save owl-bt.json
        with open(path, "w") as jsonFile:
          json.dump(data, jsonFile)

        return self.success("owl-bt is instantiated")


    def getSkills(self):
        self._get_skills = rospy.ServiceProxy('/infraflex_robot/get_skills', srvs.ResourceGetDescriptions)
        self._skill_list = dict()

        msg = srvs.ResourceGetDescriptionsRequest()
        try:
            resp1 = self._get_skills(msg)
            res = resp1
        except rospy.ServiceException as e:
            log.error("[call]", "Service call failed: %s"%e)
            return
        if not res:
                log.error("[{}]".format(self.__class__.__name__), "Can t retrieve skills.")
        else:
            return res.list

 
