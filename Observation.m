classdef Observation < handle
    properties
        agent  
        t
        score
    end
    
    
    methods
    
        function setObservation(self,env,globalview)
            self.agent=env.cartpole;
            self.t=env.t;
            self.score=env.score.score;
        end


        function self = Observation()
        end

       
    end
end