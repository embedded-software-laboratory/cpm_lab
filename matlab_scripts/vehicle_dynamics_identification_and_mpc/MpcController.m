classdef MpcController
    %MPCCONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        control
        parameters
    end
    
    methods
        function obj = MpcController(parameters)
            obj.parameters = parameters;            
        end
        
        function control = update(obj, state)
            
            obj.control = [0,0,8];
            control = obj.control;
        
        end
    end
end

