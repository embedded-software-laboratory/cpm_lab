% Copyright (c) 2014, Brian Moore
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% * Redistributions of source code must retain the above copyright notice, this
%   list of conditions and the following disclaimer.
%
% * Redistributions in binary form must reproduce the above copyright notice,
%   this list of conditions and the following disclaimer in the documentation
%   and/or other materials provided with the distribution
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
% FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
% DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
% SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
% OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
% OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%

classdef RedBlackTree < handle
%--------------------------------------------------------------------------
% Class:        RedBlackTree < handle
%               
% Constructor:  T = RedBlackTree();
%               
% Properties:   (none)
%               
% Methods:              T.Insert(key,value);
%                       T.Delete(x);
%               key   = T.Sort();
%               bool  = T.ContainsKey(key);
%               x     = T.Search(key);
%               x     = T.Minimum();
%               y     = T.NextSmallest(x);
%               x     = T.Maximum();
%               y     = T.NextLargest(x);
%               x     = T.Select(i);
%               r     = T.Rank(x);
%               count = T.Count();
%               bool  = T.IsEmpty();
%                       T.Clear();
%               
% Description:  This class implements a red-black binary search tree with
%               numeric keys and arbitrarily typed values
%               
% Author:       Brian Moore
%               brimoor@umich.edu
%               
% Date:         January 16, 2014
%--------------------------------------------------------------------------
    %
    % Private properties
    %
    properties (Access = private)
        k;                       % current number of elements
        root;                    % tree root pointer
        nil;                     % nil element pointer
    end
    
    %
    % Public methods
    %
    methods (Access = public)
        %
        % Constructor
        %
        function this = RedBlackTree()
            %----------------------- Constructor --------------------------
            % Syntax:       T = RedBlackTree();
            %               
            % Description:  Creates an empty red-black binary search tree
            %--------------------------------------------------------------
            
            % Start with an empty tree
            this.Clear();
        end
        
        %
        % Create element from key-value pair and insert into tree
        %
        function Insert(this,key,value)
            %------------------------- Insert -----------------------------
            % Syntax:       T.Insert(key,value);
            %               
            % Inputs:       key is a numeric key
            %               
            %               value is an arbitrary object
            %               
            % Description:  Inserts the given key-value pair into T
            %--------------------------------------------------------------
            
            % Create element from data
            if (nargin == 2)
                x = RBTElement(key);
            else
                x = RBTElement(key,value);
            end
            x.size = 1; % Initialize element size
            this.k = this.k + 1;
            
            % Insert element
            y = this.nil;
            z = this.root;
            while ~isnan(z)
                y = z;
                z.size = z.size + 1; % Update size
                if (x.key < z.key)
                    z = z.left;
                else
                    z = z.right;
                end
            end
            x.p = y;
            if isnan(y)
                this.root = x;
            elseif (x.key < y.key)
                y.left = x;
            else
                y.right = x;
            end
            x.left = this.nil;
            x.right = this.nil;
            x.color = true;
            
            % Clean-up tree colors after insertion
            this.InsertFixup(x);
        end
        
        %
        % Delete element from tree
        %
        function Delete(this,x)
            %------------------------- Delete -----------------------------
            % Syntax:       T.Delete(x);
            %               
            % Inputs:       x is a node in T (i.e., an object of type
            %               RBTElement) presumably extracted via a
            %               prior operation on T
            %               
            % Description:  Deletes the node x from T
            %--------------------------------------------------------------
            
            if ~isnan(x)
                % Update length
                this.k = this.k - 1;
                
                % Delete element
                orig_color = x.color;
                if isnan(x.left)
                    z = x.right;
                    this.Transplant(x,x.right);
                    this.TraverseUpwardPath(x.p); % Update size
                elseif isnan(x.right)
                    z = x.left;
                    this.Transplant(x,x.left);
                    this.TraverseUpwardPath(x.p); % Update size
                else
                    y = RedBlackTree.TreeMinimum(x.right);
                    this.TraverseUpwardPath(y.p); % Update size
                    orig_color = y.color;
                    z = y.right;
                    if (y.p == x)
                        z.p = y;
                    else
                        this.Transplant(y,y.right);
                        y.right = x.right;
                        y.right.p = y;
                    end
                    this.Transplant(x,y);
                    y.left = x.left;
                    y.left.p = y;
                    y.color = x.color;
                    y.size = y.left.size + y.right.size + 1; % Update size
                end
                
                % Clean-up tree colors after deletion, if necessary
                if (orig_color == false)
                    this.DeleteFixup(z);
                end
            end
        end
        
        %
        % Return array of sorted keys
        %
        function keys = Sort(this)
            %-------------------------- Sort ------------------------------
            % Syntax:       keys = T.Sort();
            %               
            % Outputs:      keys is a column vector containing the sorted
            %               (ascending order) keys contained in T
            %               
            % Description:  Deletes the node x from T
            %--------------------------------------------------------------
            
            keys = RBTElement(1,zeros(this.k,1));
            RedBlackTree.InOrderTreeWalk(this.root,keys);
            keys = keys.value;
        end
        
        %
        % Determine if tree contains element with given key
        %
        function bool = ContainsKey(this,key)
            %---------------------- ContainsKey ---------------------------
            % Syntax:       bool = T.ContainsKeys(key);
            %               
            % Inputs:       key is a numeric key
            %               
            % Outputs:      bool = {true,false}
            %               
            % Description:  Determines if T contains an element with the
            %               given key
            %--------------------------------------------------------------
            
            bool = ~isnan(this.Search(key));
        end
        
        %
        % Return element with given key
        %
        function x = Search(this,key)
            %------------------------- Search -----------------------------
            % Syntax:       x = T.Search(key);
            %               
            % Inputs:       key is a numeric key
            %               
            % Outputs:      x is a node from T (i.e., an object of class
            %               RBTElement) with the given key, if it exists in
            %               T, and NaN otherwise
            %               
            % Description:  Returns node from T with the given key
            %--------------------------------------------------------------
            
            x = this.root;
            while (~isnan(x) && (key ~= x.key))
                if (key < x.key)
                    x = x.left;
                else
                    x = x.right;
                end
            end
        end
        
        %
        % Return pointer to element with smallest key
        %
        function x = Minimum(this)
            %------------------------- Minimum ----------------------------
            % Syntax:       x = T.Minimum();
            %               
            % Outputs:      x is the node from T (i.e., an object of class
            %               RBTElement) with the smallest key
            %               
            % Description:  Returns node from T with the smallest key
            %--------------------------------------------------------------
            
            x = RedBlackTree.TreeMinimum(this.root);
        end
        
        %
        % Return element with next smallest key (predecessor) of x
        %
        function y = NextSmallest(this,x) %#ok
            %---------------------- NextSmallest --------------------------
            % Syntax:       y = T.NextSmallest(x);
            %               
            % Inputs:       x is a node from T (i.e., an object of class
            %               RBTElement)
            %               
            % Outputs:      y is the node from T (i.e., an object of class
            %               RBTElement) with the next smallest key than x
            %               
            % Description:  Returns node from T with the next smallest key
            %               than the input node
            %--------------------------------------------------------------
            
            if ~isnan(x)
                if ~isnan(x.left)
                    y = RedBlackTree.TreeMaximum(x.left);
                else
                    y = x.p;
                    while (~isnan(y) && (x == y.left))
                        x = y;
                        y = y.p;
                    end
                end
            else
                y = nan;
            end
        end
        
        %
        % Return pointer to element with largest key
        %
        function x = Maximum(this)
            %------------------------- Maximum ----------------------------
            % Syntax:       x = T.Maximum();
            %               
            % Outputs:      x is the node from T (i.e., an object of class
            %               RBTElement) with the largest key
            %               
            % Description:  Returns node from T with the largest key
            %--------------------------------------------------------------
            
            x = RedBlackTree.TreeMaximum(this.root);
        end
        
        %
        % Return element with next largest key (successor) of x
        %
        function y = NextLargest(this,x) %#ok
            %---------------------- NextLargest ---------------------------
            % Syntax:       y = T.NextLargest(x);
            %               
            % Inputs:       x is a node from T (i.e., an object of class
            %               RBTElement)
            %               
            % Outputs:      y is the node from T (i.e., an object of class
            %               RBTElement) with the next largest key than x
            %               
            % Description:  Returns node from T with the next largest key
            %               than the input node
            %--------------------------------------------------------------
            
            if ~isnan(x)
                if ~isnan(x.right)
                    y = RedBlackTree.TreeMinimum(x.right);
                else
                    y = x.p;
                    while (~isnan(y) && (x == y.right))
                        x = y;
                        y = y.p;
                    end
                end
            else
                y = nan;
            end
        end
        
        %
        % Return pointer to element with ith smallest key
        %
        function x = Select(this,i)
            %-------------------------- Select ----------------------------
            % Syntax:       x = T.Select(i);
            %               
            % Inputs:       i is a positive integer
            %               
            % Outputs:      x is the node from T (i.e., an object of class
            %               RBTElement) with ith smallest key
            %               
            % Description:  Returns node from T with the ith smallest key
            %--------------------------------------------------------------
            
            x = RedBlackTree.TreeSelect(this.root,i);
        end
        
        %
        % Return the rank (sorted index) of given element in the tree
        %
        function r = Rank(this,x)
            %--------------------------- Rank -----------------------------
            % Syntax:       r = T.Rank(x);
            %               
            % Inputs:       x is the node from T (i.e., an object of class
            %               RBTElement) with ith smallest key
            %               
            % Outputs:      r is the postivie integer such that x's key is
            %               the rth smallest in T
            %               
            % Description:  Returns the rank of node x in T
            %--------------------------------------------------------------
            
            r = x.left.size + 1;
            y = x;
            while (y ~= this.root)
                if (y == y.p.right)
                    r = r + y.p.left.size + 1;
                end
                y = y.p;
            end
        end
        
        %
        % Return number of elements in tree
        %
        function count = Count(this)
            %-------------------------- Count -----------------------------
            % Syntax:       count = T.Count();
            %               
            % Outputs:      count is the number of nodes in T
            %               
            % Description:  Returns number of elements in T
            %--------------------------------------------------------------
            
            count = this.k;
        end
        
        %
        % Check for empty tree
        %
        function bool = IsEmpty(this)
            %------------------------ IsEmpty -----------------------------
            % Syntax:       bool = T.IsEmpty();
            %               
            % Outputs:      bool = {true,false}
            %               
            % Description:  Determines if T is empty (i.e., contains zero
            %               elements)
            %--------------------------------------------------------------
            
            if (this.k == 0)
                bool = true;
            else
                bool = false;
            end
        end
        
        %
        % Clear the tree
        %
        function Clear(this)
            %------------------------- Clear ------------------------------
            % Syntax:       T.Clear();
            %               
            % Description:  Removes all elements from T
            %--------------------------------------------------------------
            
            this.k = 0;                     % reset length counter
            this.nil = RBTElement(nan);     % reset nil pointer
            this.root = this.nil;           % reset root pointer
        end
    end
    
    %
    % Private methods
    %
    methods (Access = private)
        %
        % Replaces the subtree rooted at element u with the subtree
        % rooted at element v
        %
        function Transplant(this,u,v)
            if isnan(u.p)
                this.root = v;
            elseif (u == u.p.left)
                u.p.left = v;
            else
                u.p.right = v;
            end
            v.p = u.p;
        end
        
        %
        % Left rotate at element x
        %
        function LeftRotate(this,x)
            y = x.right;
            x.right = y.left;
            if ~isnan(y.left)
                y.left.p = x;
            end
            y.p = x.p;
            if isnan(x.p)
                this.root = y;
            elseif (x == x.p.left)
                x.p.left = y;
            else
                x.p.right = y;
            end
            y.left = x;
            x.p = y;
            y.size = x.size; % Update size
            x.size = x.left.size + x.right.size + 1; % Update size
        end
        
        %
        % Right rotate at element x
        %
        function RightRotate(this,x)
            y = x.left;
            x.left = y.right;
            if ~isnan(y.right)
                y.right.p = x;
            end
            y.p = x.p;
            if isnan(x.p)
                this.root = y;
            elseif (x == x.p.right)
                x.p.right = y;
            else
                x.p.left = y;
            end
            y.right = x;
            x.p = y;
            y.size = x.size; % Update size
            x.size = x.left.size + x.right.size + 1; % Update size
        end
        
        %
        % Fix tree coloring after inserting element x
        %
        function InsertFixup(this,x)
            while (x.p.color == true)
                if (x.p == x.p.p.left)
                    y = x.p.p.right;
                    if (y.color == true)
                        x.p.color = false;
                        y.color = false;
                        x.p.p.color = true;
                        x = x.p.p;
                    else
                        if (x == x.p.right)
                            x = x.p;
                            this.LeftRotate(x);
                        end
                        x.p.color = false;
                        x.p.p.color = true;
                        this.RightRotate(x.p.p);
                    end
                else
                    y = x.p.p.left;
                    if (y.color == true)
                        x.p.color = false;
                        y.color = false;
                        x.p.p.color = true;
                        x = x.p.p;
                    else
                        if (x == x.p.left)
                            x = x.p;
                            this.RightRotate(x);
                        end
                        x.p.color = false;
                        x.p.p.color = true;
                        this.LeftRotate(x.p.p);
                    end
                end
            end
            this.root.color = false;
        end
        
        %
        % Fix tree coloring after deleting element x, which necessitates
        % calling this function with parameter z, as defined in Delete()
        %
        function DeleteFixup(this,z)
            while ((z ~= this.root) && (z.color == false))
                if (z == z.p.left)
                    w = z.p.right;
                    if (w.color == true)
                        w.color = false;
                        z.p.color = true;
                        this.LeftRotate(z.p);
                        w = z.p.right;
                    end
                    if ((w.left.color == false) && (w.right.color == false))
                        w.color = true;
                        z = z.p;
                    else
                        if (w.right.color == false)
                            w.left.color = false;
                            w.color = true;
                            this.RightRotate(w);
                            w = z.p.right;
                        end
                        w.color = z.p.color;
                        z.p.color = false;
                        w.right.color = false;
                        this.LeftRotate(z.p);
                        z = this.root;
                    end
                else
                    w = z.p.left;
                    if (w.color == true)
                        w.color = false;
                        z.p.color = true;
                        this.RightRotate(z.p);
                        w = z.p.left;
                    end
                    if ((w.left.color == false) && (w.right.color == false))
                        w.color = true;
                        z = z.p;
                    else
                        if (w.left.color == false)
                            w.right.color = false;
                            w.color = true;
                            this.LeftRotate(w);
                            w = z.p.left;
                        end
                        w.color = z.p.color;
                        z.p.color = false;
                        w.left.color = false;
                        this.RightRotate(z.p);
                        z = this.root;
                    end
                end
            end
            z.color = false;
        end
        
        %
        % Traverse path from y towards the root, decrementing sizes along
        % the way
        %
        function TraverseUpwardPath(this,y)
            if ~isnan(y)
                y.size = y.size - 1; % Update size
                if (y ~= this.root)
                    this.TraverseUpwardPath(y.p);
                end
            end
        end
    end
    
    %
    % Private static methods
    %
    methods (Access = private, Static = true)
        %
        % In-order tree walk from given element
        %
        function InOrderTreeWalk(x,keys)
            if ~isnan(x)
                RedBlackTree.InOrderTreeWalk(x.left,keys);
                keys.value(keys.key) = x.key;
                keys.key = keys.key + 1;
                RedBlackTree.InOrderTreeWalk(x.right,keys);
            end
        end
        
        %
        % Return pointer to minimum of subtree rooted at x
        %
        function x = TreeMinimum(x)
            if ~isnan(x)
                while ~isnan(x.left)
                    x = x.left;
                end
            end
        end
        
        %
        % Return pointer to maximum of subtree rooted at x
        %
        function x = TreeMaximum(x)
            if ~isnan(x)
                while ~isnan(x.right)
                    x = x.right;
                end
            end
        end
        
        %
        % Returns pointer to element with ith smallest key in the subtree
        % rooted at element x
        %
        function x = TreeSelect(x,i)
            if ~isnan(x)
                r = x.left.size + 1;
                if (i < r)
                    x = RedBlackTree.TreeSelect(x.left,i);
                elseif (i > r)
                    x = RedBlackTree.TreeSelect(x.right,i - r);
                end
            end
        end
    end
end
