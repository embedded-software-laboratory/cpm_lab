% MIT License
% 
% Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% 
% This file is part of cpm_lab.
% 
% Author: i11 - Embedded Software, RWTH Aachen University

% A self-written implementation of a List as dynamic data structure
classdef MyList < handle
    
%     Constructor:
%      function obj= MyList(newContent):
%               call MyList() to generate a head for the list
%               call MyList(newContent) to generate an List Element with
%                   newContent
%      function obj = appendElement(obj,newMyList):
%                 
%      function removeThisElement(obj)
%      function removeByIndex(obj,index):
%                   removes the index-th successor from the list, counting
%                   from this Element
%      function result=getElement(obj,index):
%                   gets the index-th successor from the list, counting
%                   from this Element
%      function Pred = get_Pred(obj)
%      function Succ = get_Succ(obj)
%      function obj = set_Succ(obj,newSucc)
%      function content = get_Content(obj)
%      function out = hasPred(obj)
%      function out = hasSucc(obj)
%     
    

   properties (Access = private)
       Pred ;
       Succ ;
       Content;
       hasSuccElement = 0;
       hasPredElement = 0;
       % head information
       isHead = 0;
       
   end
       methods
           %% constructor
           function obj=MyList(newContent)
               switch nargin
                   case 0
                       % to create an empty root of List
                       obj.isHead=1;
                   case 1              
                      obj.Content = newContent;
                      
                   otherwise
                       error('nargin invalid in MyList');
               end
           end
           
           %% function to append Element or List of Element
           function obj = appendElement(obj,newMyList)
               if obj.hasSuccElement  == 0
                   obj.Succ = newMyList;   
                   obj.hasSuccElement = 1;
                   newMyList.set_Pred(obj);
               else
                   thisSucc=obj.Succ;
                   appendElement(thisSucc,newMyList);                   
               end
               
           end
           function removeThisElement(obj)
               if  obj.hasPred == 0
                   % root
                   obj.delete();
                   return;
               else
               
                   if obj.hasSucc 
                        thisSucc= obj.Succ;
                        thisPred = obj.Pred;
                        thisSucc.set_Pred(thisPred);
                        thisPred.set_Succ(thisSucc);
                   else
                        % tail
                        thisPred = obj.Pred;
                        thisPred.set_hasSucc = 0;
                        thisPred.set_Succ([]);                  
                   end


                   obj.delete(); % destructor
               end
           end
           function removeByIndex(obj,index)
               deletionObj = obj.getElement(index);
               removeThisElement(deletionObj);               
           end
           
           %% returns Element #index starting from obj. 0 is obj
           function result=getElement(obj,index)
        
               if index < 0
                   error('invalid index in MyList.getElement()');
               end
              if index==0
                  result=obj;
              else
                 if obj.hasSucc
                     thisSucc = obj.Succ;
                    result = thisSucc.getElement(index-1);
                 else
                      % reached end of list
                     error('indexOutOfBound in MyList.getElement');
                 end
                  
              end
           end
           
           
           %% get and set
          
           function Pred = get_Pred(obj)
               Pred = obj.Pred;
           end
           
           % sets a new predecessor. set_Pred(obj) to delete predecessor
           function obj = set_Pred(obj,newPred)
               if nargin ==2
                obj.Pred = newPred; 
                obj.hasPredElement = 1;
               else
                   obj.Pred = [];
                   obj.hasPredElement = 0;
               end
                   
           end
           function Succ = get_Succ(obj)
               Succ = obj.Succ;
           end
           % sets a new successor. set_Pred(obj) to delete successor
           function obj = set_Succ(obj,newSucc)
                if nargin ==2
                   obj.Succ= newSucc;
                   obj.hasSuccElement = 1;
               else
                   obj.Succ = [];
                   obj.hasSuccElement = 0;
               end
           end

           function out = hasSucc(obj)
               out = obj.hasSuccElement;
           end
           function out = hasPred(obj)
               out = obj.hasPredElement;
           end
           function content = get_Content(obj)
              content = obj.Content; 
           end
       end
end