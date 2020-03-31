#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <functional>
#include <iostream>
#include <string>
#include <vector>

/**
 * This is a utility namespace
 * It includes functions that are used for XML Translation throughout the commonroad_classes folder
 */

namespace xml_translation
{
    //**********************************************************************
    //Elements
    //**********************************************************************

    /**
     * \brief Takes a node as input, assuming it is of type TextNode (which is tested within the function TODO:Throw error, so that the user does not have to do it, thus xmlpp::Node, not xmlpp::TextNode was chosen)
     * Then, it gets its content in form of a string
     * \param node An XML node, assumed to be of type TextNode (which must not be checked by the user)
     * \return A string containing the content of the TextNode, or 'empty' if there is none
     */
    std::string get_node_text(const xmlpp::Node* node);

    /**
     * \brief Takes a node as input, assuming it is of type TextNode (which is tested within the function, so that the user does not have to do it, thus xmlpp::Node, not xmlpp::TextNode was chosen)
     * Then, it gets its content in form of an int, if that is possible, or else TODO: throws an error 
     * \param node An XML node, assumed to be of type TextNode (which must not be checked by the user)
     * \return An int containing the content of the TextNode, or -1 in case of an error
     */
    int get_node_int(const xmlpp::Node* node);

    /**
     * \brief Takes a node as input, assuming it is of type TextNode (which is tested within the function, so that the user does not have to do it, thus xmlpp::Node, not xmlpp::TextNode was chosen)
     * Then, it gets its content in form of an int, if that is possible, or else TODO: throws an error 
     * \param node An XML node, assumed to be of type TextNode (which must not be checked by the user)
     * \return An unsigned long long value containing the content of the TextNode, or 0 in case of an error
     */
    unsigned long long get_node_uint(const xmlpp::Node* node);

    /**
     * \brief Takes a node as input, assuming it is of type TextNode (which is tested within the function, so that the user does not have to do it, thus xmlpp::Node, not xmlpp::TextNode was chosen)
     * Then, it gets its content in form of an double, if that is possible, or else TODO: throws an error 
     * \param node An XML node, assumed to be of type TextNode (which must not be checked by the user)
     * \return A double value containing the content of the TextNode, or -1.0 in case of an error
     */
    double get_node_double(const xmlpp::Node* node);

    /**
     * \brief Takes a node as input, assuming it is of type Element (which is tested within the function, so that the user does not have to do it, thus xmlpp::Node, not xmlpp::Element was chosen)
     * Always warns if it is not an Element node TODO: throw error
     * Then, it translates all elements according to the transformation function and returns the list of all elements (of a given name, if specified)
     * \param node An XML node 
     * \param node_transform Transformation function that takes a node and translates it to a type T
     * \param child_name Expected name of the child node - optional, return all if not set
     * \return Translated list of all child nodes (with name child_name) (if it does not exist, returns empty list)
     */
    template<class T>
    std::vector<T> get_children(const xmlpp::Node* node, std::function<T (xmlpp::Node* node)> node_transfrom, std::string child_name = "")
    {
        //Check if it is an element node
        const xmlpp::Element* node_element = dynamic_cast<const xmlpp::Element*>(node);
        if(!(node_element))
        {
            std::cerr << "TODO: Better warning // Node not of expected type element: " << node->get_name() << std::endl;
            return nullptr;
        }

        xmlpp::Node::NodeList children;
        std::vector<T> transformed_children;

        //Get all children (of the given name)
        if (child_name != "")
        {
            children = node_element->get_children(child_name);
        }
        else
        {
            children = node_element->get_children();
        }

        //Transform from child list to transformed child list by translating each children to type T using the given function node_transform
        for (const auto child : children)
        {
            transformed_children.push_back(node_transfrom(child));
        }

        return transformed_children;
    }

    /**
     * \brief Takes a node as input, assuming it is of type Element (which is tested within the function, so that the user does not have to do it, thus xmlpp::Node, not xmlpp::Element was chosen)
     * Always warns if it is not an Element node TODO: throw error
     * \param node An XML node 
     * \param child_name Expected name of the child node
     * \param warn Warn if the child does not exist (if true, else stay silent) - optional TODO: throw error
     * \return Pointer to the child node (if it does not exist, returns nullptr
     */
    const xmlpp::Node* get_child_if_exists(const xmlpp::Node* node, std::string child_name, bool warn = false);

    /**
     * \brief Takes a node as input, assuming it is of type Element (which is tested within the function, so that the user does not have to do it, thus xmlpp::Node, not xmlpp::Element was chosen)
     * Then, assuming that it is one of the common commonroad element nodes like '<country>ZAM</country>', where the first child node is a text node containint 'ZAM', 
     * it checks whether this node exists, and, if so, gets its content in form of a string
     * TODO: Throw error
     * \param node An XML node, assumed to be of type Element (which must not be checked by the user)
     * \return A string containing the content, or 'empty' if there is none
     */
    std::string get_first_child_text(const xmlpp::Node* node);

    /**
     * \brief Takes a node as input, assuming it is of type Element (which is tested within the function, so that the user does not have to do it, thus xmlpp::Node, not xmlpp::Element was chosen)
     * Then, assuming that it is one of the common commonroad element nodes like '<country>ZAM</country>', where the first child node is a text node containint 'ZAM', 
     * it checks whether this node exists, and, if so, gets its content in form of an int
     * TODO: Throw error
     * \param node An XML node, assumed to be of type Element (which must not be checked by the user)
     * \return An int containing the content, or -1 in case of an error
     */
    int get_first_child_int(const xmlpp::Node* node);

    /**
     * \brief Takes a node as input, assuming it is of type Element (which is tested within the function, so that the user does not have to do it, thus xmlpp::Node, not xmlpp::Element was chosen)
     * Then, assuming that it is one of the common commonroad element nodes like '<country>ZAM</country>', where the first child node is a text node containint 'ZAM', 
     * it checks whether this node exists, and, if so, gets its content in form of an unsigned long long
     * TODO: Throw error
     * \param node An XML node, assumed to be of type Element (which must not be checked by the user)
     * \return A uint containing the content, or 0 in case of an error
     */
    unsigned long long get_first_child_uint(const xmlpp::Node* node);

    /**
     * \brief Takes a node as input, assuming it is of type Element (which is tested within the function, so that the user does not have to do it, thus xmlpp::Node, not xmlpp::Element was chosen)
     * Then, assuming that it is one of the common commonroad element nodes like '<country>ZAM</country>', where the first child node is a text node containint 'ZAM', 
     * it checks whether this node exists, and, if so, gets its content in form of a double
     * TODO: Throw error
     * \param node An XML node, assumed to be of type Element (which must not be checked by the user)
     * \return A double containing the content, or -1.0 in case of an error
     */
    double get_first_child_double(const xmlpp::Node* node);

    /**
     * \brief Takes a node as input, assuming it is of type Element (which is tested within the function, so that the user does not have to do it, thus xmlpp::Node, not xmlpp::Element was chosen)
     * Always warns if it is not an Element node TODO: throw error
     * \param element_node An XML node
     * \param child_name Expected name of the child node
     * \param warn Warn if the child does not exist (if true, else stay silent) - optional TODO: throw error
     * \return Value of the first child-child of the first child with name child_name of node, or 'empty' in case of an error
     */
    std::string get_child_child_text(const xmlpp::Node* node, std::string child_name, bool warn = false);

    /**
     * \brief Takes a node as input, assuming it is of type Element (which is tested within the function, so that the user does not have to do it, thus xmlpp::Node, not xmlpp::Element was chosen)
     * Always warns if it is not an Element node TODO: throw error
     * \param element_node An XML node
     * \param child_name Expected name of the child node
     * \param warn Warn if the child does not exist (if true, else stay silent) - optional TODO: throw error
     * \return Value of the first child-child of the first child with name child_name of node, or -1 in case of an error
     */
    int get_child_child_int(const xmlpp::Node* node, std::string child_name, bool warn = false);

    /**
     * \brief Takes a node as input, assuming it is of type Element (which is tested within the function, so that the user does not have to do it, thus xmlpp::Node, not xmlpp::Element was chosen)
     * Always warns if it is not an Element node TODO: throw error
     * \param element_node An XML node
     * \param child_name Expected name of the child node
     * \param warn Warn if the child does not exist (if true, else stay silent) - optional TODO: throw error
     * \return Value of the first child-child of the first child with name child_name of node, or 0 in case of an error
     */
    unsigned long long get_child_child_uint(const xmlpp::Node* node, std::string child_name, bool warn = false);

    /**
     * \brief Takes a node as input, assuming it is of type Element (which is tested within the function, so that the user does not have to do it, thus xmlpp::Node, not xmlpp::Element was chosen)
     * Always warns if it is not an Element node TODO: throw error
     * \param element_node An XML node
     * \param child_name Expected name of the child node
     * \param warn Warn if the child does not exist (if true, else stay silent) - optional TODO: throw error
     * \return Value of the first child-child of the first child with name child_name of node, or -1.0 in case of an error
     */
    double get_child_child_double(const xmlpp::Node* node, std::string child_name, bool warn = false);

    //**********************************************************************
    //Attributes
    //**********************************************************************

    /**
     * \brief Takes a node as input, assuming it is of type Element (which is tested within the function, so that the user does not have to do it, thus xmlpp::Node, not xmlpp::Element was chosen)
     * Always warns if it is not an Element node TODO: throw error
     * Then, it translates the attribute of all elements with the given attribute according to the transformation function and returns the list of all resulting ojects T (of a given name, if specified)
     * Example: <a ref="1"/> <a ref="2"/> -> [1, 2]
     * \param node An XML node 
     * \param attribute_transform Transformation function that takes an attribute and translates it to a type T
     * \param node_name Expected name of the node
     * \param attribute_name Expected name of the attribute
     * \return Translated list of all child nodes (with name child_name) (if it does not exist, returns empty list)
     */
    template<class T>
    std::vector<T> get_elements_with_attribute(const xmlpp::Node* node, std::function<T (std::string)> attribute_transform, std::string node_name, std::string attribute_name)
    {
        //Check if it is an element node
        const xmlpp::Element* node_element = dynamic_cast<const xmlpp::Element*>(node);
        if(!(node_element))
        {
            std::cerr << "TODO: Better warning // Node not of expected type element: " << node->get_name() << std::endl;
            return nullptr;
        }

        xmlpp::Node::NodeList children;
        std::vector<T> transformed_attributes;

        //Get all children (of the given name)
        if (child_name != "")
        {
            children = node_element->get_children(child_name);
        }
        else
        {
            children = node_element->get_children();
        }

        //Transform from child list to transformed child list by translating each children to type T using the given function node_transform
        for (const auto child : children)
        {
            transformed_attributes.push_back(node_transfrom(xml_translation::get_attribute_text(child, attribute_name, true)));
        }

        return transformed_attributes;
    }

    /**
     * \brief Takes a node as input, assuming it is of type Element (which is tested within the function TODO:Throw error, so that the user does not have to do it, thus xmlpp::Node, not xmlpp::Element was chosen)
     * Then, it gets its content in form of a string
     * Shows /throws an error if desired, always warns if node is not of type Element
     * \param node An XML node, assumed to be of type Element (which must not be checked by the user)
     * \param attribute_name Name of the attribute of which to get the value
     * \param warn Optional parameter, warn / throw error if the attribute does not exists only if desired (it might not be required by specs)
     * \return A string containing the content of the Element, or 'empty' if there is none
     */
    std::string get_attribute_text(const xmlpp::Node* node, std::string attribute_name, bool warn = false);

    /**
     * \brief Takes a node as input, assuming it is of type Element (which is tested within the function, so that the user does not have to do it, thus xmlpp::Node, not xmlpp::Element was chosen)
     * Then, it gets its content in form of an int, if that is possible, or else TODO: throws an error 
     * Shows /throws an error if desired, always warns if node is not of type Element
     * \param node An XML node, assumed to be of type Element (which must not be checked by the user)
     * \param attribute_name Name of the attribute of which to get the value
     * \param warn Optional parameter, warn / throw error if the attribute does not exists only if desired (it might not be required by specs)
     * \return An int containing the content of the Element, or -1 in case of an error
     */
    int get_attribute_int(const xmlpp::Node* node, std::string attribute_name, bool warn = false);

    /**
     * \brief Takes a node as input, assuming it is of type Element (which is tested within the function, so that the user does not have to do it, thus xmlpp::Node, not xmlpp::Element was chosen)
     * Then, it gets its content in form of an unsigned long long, if that is possible, or else TODO: throws an error 
     * Shows /throws an error if desired, always warns if node is not of type Element
     * \param node An XML node, assumed to be of type Element (which must not be checked by the user)
     * \param attribute_name Name of the attribute of which to get the value
     * \param warn Optional parameter, warn / throw error if the attribute does not exists only if desired (it might not be required by specs)
     * \return An unsigned long long value containing the content of the Element, or 0 in case of an error
     */
    unsigned long long get_attribute_uint(const xmlpp::Node* node, std::string attribute_name, bool warn = false);

    /**
     * \brief Takes a node as input, assuming it is of type Element (which is tested within the function, so that the user does not have to do it, thus xmlpp::Node, not xmlpp::Element was chosen)
     * Then, it gets its content in form of a double, if that is possible, or else TODO: throws an error 
     * Shows /throws an error if desired, always warns if node is not of type Element
     * \param node An XML node, assumed to be of type Element (which must not be checked by the user)
     * \param attribute_name Name of the attribute of which to get the value
     * \param warn Optional parameter, warn / throw error if the attribute does not exists only if desired (it might not be required by specs)
     * \return An double value containing the content of the Element, or -1.0 in case of an error
     */
    double get_attribute_double(const xmlpp::Node* node, std::string attribute_name, bool warn = false);

    //**********************************************************************
    //Helper functions
    //**********************************************************************

    /**
     * \brief Gets a string TODO: throws an error (if desired)
     * Transforms the string to an int, TODO: throws an error (if desired) if no transformation is possible
     * \param text A string that represents an int value
     * \param warn Only warn if set to true, optional
     * \return The int value of the string, else -1
     */
    int string_to_int(std::string text, bool warn = false);

    /**
     * \brief Gets a string TODO: throws an error (if desired)
     * Transforms the string to an unsigned long long, TODO: throws an error (if desired) if no transformation is possible
     * \param text A string that represents an unsigned long long value
     * \param warn Only warn if set to true, optional
     * \return The unsigned long long value of the string, else 0
     */
    unsigned long long string_to_uint(std::string text, bool warn = false);

    /**
     * \brief Gets a string TODO: throws an error (if desired)
     * Transforms the string to an double, TODO: throws an error (if desired) if no transformation is possible
     * \param text A string that represents an double value
     * \param warn Only warn if set to true, optional
     * \return The double value of the string, else -1.0
     */
    double string_to_double(std::string text, bool warn = false);
}