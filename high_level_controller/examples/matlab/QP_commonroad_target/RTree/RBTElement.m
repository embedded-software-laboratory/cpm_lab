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

classdef RBTElement < handle
%
% A class that implements pass by reference elements with numeric keys for
% instances of the RedBlackTree class
%
% NOTE: This class is used internally by the RedBlackTree class
%
    %
    % Public properties
    %
	properties (Access = public)
		key;            % key
        left = nan      % left child
        right = nan;    % right child
        p = nan;        % parent
        color = false;  % color (true = red,false = black)
        size = 0;       % size of subtree rooted at this element
        value = [];     % miscellaneous data
    end
    
    %
    % Public methods
    %
	methods (Access = public)
        %
        % Constructor
        %
		function this = RBTElement(key,value)
			% Initialize key
            this.key = key;
            
            % Set value data, if specified
            if (nargin == 2)
                this.value = value;
            end
        end
        
        %
        % Element is nan if its key is nan
        %
        function bool = isnan(this)
            bool = isnan(this.key);
        end
    end
end
