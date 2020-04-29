#include "XMLTranslation.hpp"


std::string xml_translation::get_node_text(const xmlpp::Node* node)
{
    //Convert to text node and check if it really exists
    const xmlpp::TextNode* text_node = dynamic_cast<const xmlpp::TextNode*>(node);
    if(!(text_node))
    {
        //TODO: Throw error
        std::stringstream error_msg_stream;
        error_msg_stream << "Node " << node->get_name() << " not of expected type TextNode, line: " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }
    
    //If it exists, return the content of the text node
    return text_node->get_content();
}

int xml_translation::get_node_int(const xmlpp::Node* node)
{
    //Get the content of the node as string, then convert it to an integer
    try
    {
        return std::stoi(xml_translation::get_node_text(node));
    }
    catch(...)
    {
        //TODO: Catch error of get_node_text and check if former return values were used in other functions
        std::stringstream error_msg_stream;
        error_msg_stream << "Node " << node->get_name() << " could not be translated to int, line: " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }
}

unsigned long long xml_translation::get_node_uint(const xmlpp::Node* node)
{
    //Get the content of the node as string, then convert it to a ull
    try
    {
        return std::stoull(xml_translation::get_node_text(node));
    }
    catch(...)
    {
        //TODO: Catch error of get_node_text and check if former return values were used in other functions
        std::stringstream error_msg_stream;
        error_msg_stream << "Node " << node->get_name() << " could not be translated to unsigned long long, line: " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }
}

double xml_translation::get_node_double(const xmlpp::Node* node)
{
    //Get the content of the node as string, then convert it to a double
    try
    {
        return std::stod(xml_translation::get_node_text(node));
    }
    catch(...)
    {
        //TODO: Catch error of get_node_text and check if former return values were used in other functions
        std::stringstream error_msg_stream;
        error_msg_stream << "Node " << node->get_name() << " could not be translated to double, line: " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }
}

const xmlpp::Node* xml_translation::get_child_if_exists(const xmlpp::Node* node, std::string child_name, bool warn)
{
    //Check if it is an element node
    const xmlpp::Element* node_element = dynamic_cast<const xmlpp::Element*>(node);
    if(!(node_element))
    {
        std::stringstream error_msg_stream;
        error_msg_stream << "Node " << node->get_name() << " not of expected type 'element' (XML 'structure' type, not commonroad), line: " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }

    //Check if the required child exists
    const xmlpp::Node* child = node_element->get_first_child(child_name);
    if (!child)
    {
        if (warn)
        {
            //TODO: Does this make sense? Also: rename warn to throw_error or something similar
            std::stringstream error_msg_stream;
            error_msg_stream << "Node " << node->get_name() << " does not have the required child '" << child_name << "', line: " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
        return nullptr;
    }

    return child;
}

std::string xml_translation::get_first_child_text(const xmlpp::Node* node)
{
    //Find out which object we are dealing with, pass on translation to these objects (if possible)
    const xmlpp::Element* node_element = dynamic_cast<const xmlpp::Element*>(node);

    if(!(node_element))
    {
        std::stringstream error_msg_stream;
        error_msg_stream << "Node " << node->get_name() << " not of expected type 'element' (XML 'structure' type, not commonroad), line: " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }
    if (!(node_element->get_first_child()))
    {
        std::stringstream error_msg_stream;
        error_msg_stream << "Node " << node->get_name() << " should have a child, but has none (XML 'structure' reason, not directly bc. of commonroad), line: " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }

    return get_node_text(node_element->get_first_child());
}

int xml_translation::get_first_child_int(const xmlpp::Node* node)
{
    //Get the content of the node as string, then convert it to an integer
    try
    {
        return std::stoi(xml_translation::get_first_child_text(node));
    }
    catch(...)
    {
        //TODO: Catch error of get_node_text and check if former return values were used in other functions
        std::stringstream error_msg_stream;
        error_msg_stream << "Node " << node->get_name() << " could not be translated to int, line: " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }
}

unsigned long long xml_translation::get_first_child_uint(const xmlpp::Node* node)
{
    //Get the content of the node as string, then convert it to an unsigned long long
    try
    {
        return std::stoull(xml_translation::get_first_child_text(node));
    }
    catch(...)
    {
        //TODO: Catch error of get_node_text and check if former return values were used in other functions
        std::stringstream error_msg_stream;
        error_msg_stream << "Node " << node->get_name() << " could not be translated to unsigned long long, line: " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }
}

double xml_translation::get_first_child_double(const xmlpp::Node* node)
{
    //Get the content of the node as string, then convert it to a double
    try
    {
        return std::stod(xml_translation::get_first_child_text(node));
    }
    catch(...)
    {
        //TODO: Catch error of get_node_text and check if former return values were used in other functions
        std::stringstream error_msg_stream;
        error_msg_stream << "Node " << node->get_name() << " could not be translated to double, line: " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }
}

std::string xml_translation::get_child_child_text(const xmlpp::Node* node, std::string child_name, bool warn)
{
    const auto child = xml_translation::get_child_if_exists(node, child_name, warn);

    if (!child)
        return "empty";
    
    return xml_translation::get_first_child_text(child);
}

int xml_translation::get_child_child_int(const xmlpp::Node* node, std::string child_name, bool warn)
{
    //Get the content of the node as string, then convert it to an integer
    try
    {
        return std::stoi(xml_translation::get_child_child_text(node, child_name, warn));
    }
    catch(...)
    {
        if (warn)
        {
            //TODO: Does this make sense? Also: rename warn to throw_error or something similar; also, catch previous errors properly / propagate them
            std::stringstream error_msg_stream;
            error_msg_stream << "Node " << node->get_name() << " - child '" << child_name << "' could not be translated to int, line: " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
        return -1;
    }
}

unsigned long long xml_translation::get_child_child_uint(const xmlpp::Node* node, std::string child_name, bool warn)
{
    //Get the content of the node as string, then convert it to an integer
    try
    {
        return std::stoull(xml_translation::get_child_child_text(node, child_name, warn));
    }
    catch(...)
    {
        if (warn)
        {
            //TODO: Does this make sense? Also: rename warn to throw_error or something similar; also, catch previous errors properly / propagate them
            std::stringstream error_msg_stream;
            error_msg_stream << "Node " << node->get_name() << " - child '" << child_name << "' could not be translated to unsigned long long, line: " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
        return 0;
    }
}

double xml_translation::get_child_child_double(const xmlpp::Node* node, std::string child_name, bool warn)
{
    //Get the content of the node as string, then convert it to an integer
    try
    {
        return std::stod(xml_translation::get_child_child_text(node, child_name, warn));
    }
    catch(...)
    {
        if (warn)
        {
            //TODO: Does this make sense? Also: rename warn to throw_error or something similar; also, catch previous errors properly / propagate them
            std::stringstream error_msg_stream;
            error_msg_stream << "Node " << node->get_name() << " - child '" << child_name << "' could not be translated to double, line: " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
        return -1.0;
    }
}

double xml_translation::get_child_child_double_exact(const xmlpp::Node* node, std::string child_name, bool warn)
{
    //Get the content of the node as string, then convert it to an integer
    try
    {
        const auto child_node = xml_translation::get_child_if_exists(node, child_name, warn);
        if (child_node)
        {
            return std::stod(xml_translation::get_child_child_text(child_node, "exact", warn));
        }
        else
        {
            //TODO: Use optional here as well, return optional? 
            if (warn)
            {
                //TODO: Does this make sense? Also: rename warn to throw_error or something similar; also, catch previous errors properly / propagate them
                std::stringstream error_msg_stream;
                error_msg_stream << "Node " << node->get_name() << " - child '" << child_name << "' could not be translated to exact double (child missing), line: " << node->get_line();
                throw SpecificationError(error_msg_stream.str());
            }
            return -1;
        }
        
    }
    catch(...)
    {
        if (warn)
        {
            //TODO: Does this make sense? Also: rename warn to throw_error or something similar; also, catch previous errors properly / propagate them
            std::stringstream error_msg_stream;
            error_msg_stream << "Node " << node->get_name() << " - child '" << child_name << "' could not be translated to exact double, line: " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
        return -1;
    }
}

std::string xml_translation::get_attribute_text(const xmlpp::Node* node, std::string attribute_name, bool warn)
{
    //Convert to text node and check if it really exists
    const xmlpp::Element* node_element = dynamic_cast<const xmlpp::Element*>(node);
    if(!(node_element))
    {
        std::stringstream error_msg_stream;
        error_msg_stream << "Node " << node->get_name() << " not of expected type 'element' (XML 'structure' type, not commonroad), line: " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }
    
    //Get the attribute and check if it exists
    const auto attribute = node_element->get_attribute(attribute_name);
    if (!attribute)
    {
        if (warn)
        {
            //TODO: Does this make sense? Also: rename warn to throw_error or something similar; also, catch previous errors properly / propagate them
            std::stringstream error_msg_stream;
            error_msg_stream << "Node " << node->get_name() << " attribute does not exist ('" << attribute_name << "'), line: " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
        return "empty";
    }

    return attribute->get_value();
}

int xml_translation::get_attribute_int(const xmlpp::Node* node, std::string attribute_name, bool warn)
{
    //Get the content of the node as string, then convert it to an integer
    try
    {
        return std::stoi(xml_translation::get_attribute_text(node, attribute_name, warn));
    }
    catch(...)
    {
        if (warn)
        {
            //TODO: Does this make sense? Also: rename warn to throw_error or something similar; also, catch previous errors properly / propagate them
            std::stringstream error_msg_stream;
            error_msg_stream << "Node " << node->get_name() << " - attribute '" << attribute_name << "' could not be translated to int, line: " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
        return -1;
    }
}

unsigned long long xml_translation::get_attribute_uint(const xmlpp::Node* node, std::string attribute_name, bool warn)
{
    //Get the content of the node as string, then convert it to a ull
    try
    {
        return std::stoull(xml_translation::get_attribute_text(node, attribute_name, warn));
    }
    catch(...)
    {
        if (warn)
        {
            //TODO: Does this make sense? Also: rename warn to throw_error or something similar; also, catch previous errors properly / propagate them
            std::stringstream error_msg_stream;
            error_msg_stream << "Node " << node->get_name() << " - attribute '" << attribute_name << "' could not be translated to uint, line: " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
        return 0;
    }
}

double xml_translation::get_attribute_double(const xmlpp::Node* node, std::string attribute_name, bool warn)
{
    //Get the content of the node as string, then convert it to a double
    try
    {
        return std::stod(xml_translation::get_attribute_text(node, attribute_name, warn));
    }
    catch(...)
    {
        if (warn)
        {
            //TODO: Does this make sense? Also: rename warn to throw_error or something similar; also, catch previous errors properly / propagate them
            std::stringstream error_msg_stream;
            error_msg_stream << "Node " << node->get_name() << " - attribute '" << attribute_name << "' could not be translated to double, line: " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
        return -1.0;
    }
}

int xml_translation::string_to_int(std::string text)
{
    try
    {
        return std::stoi(text);
    }
    catch(...)
    {
        std::cerr << "TODO: Better warning // Could not translate text to int" << std::endl;
        return -1;
    }
}

unsigned long long xml_translation::string_to_uint(std::string text)
{
    try
    {
        return std::stoull(text);
    }
    catch(...)
    {
        std::cerr << "TODO: Better warning // Could not translate text to unsigned long long" << std::endl;
        return 0;
    }
}

double xml_translation::string_to_double(std::string text)
{
    try
    {
        return std::stod(text);
    }
    catch(...)
    {
        std::cerr << "TODO: Better warning // Could not translate text to double" << std::endl;
        return -1.0;
    }
}

void xml_translation::iterate_children(const xmlpp::Node* node, std::function<void (xmlpp::Node* node)> node_function, std::string child_name = "")
{
    //Check if it is an element node
    const xmlpp::Element* node_element = dynamic_cast<const xmlpp::Element*>(node);
    if(!(node_element))
    {
        std::stringstream error_msg_stream;
        error_msg_stream << "Node " << node->get_name() << " not of expected type 'element' (XML 'structure' type, not commonroad), line: " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }

    xmlpp::Node::NodeList children;

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
        node_function(child);
    }
}

void xml_translation::iterate_elements_with_attribute(const xmlpp::Node* node, std::function<void (std::string)> attribute_function, std::string node_name, std::string attribute_name)
{
    //Check if it is an element node
    const xmlpp::Element* node_element = dynamic_cast<const xmlpp::Element*>(node);
    if(!(node_element))
    {
        std::stringstream error_msg_stream;
        error_msg_stream << "Node " << node->get_name() << " not of expected type 'element' (XML 'structure' type, not commonroad), line: " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }

    xmlpp::Node::NodeList children;

    //Get all children (of the given name)
    if (node_name != "")
    {
        children = node_element->get_children(node_name);
    }
    else
    {
        children = node_element->get_children();
    }

    //Transform from child list to transformed child list by translating each children to type T using the given function node_transform
    for (const auto child : children)
    {
        attribute_function(xml_translation::get_attribute_text(child, attribute_name, true));
    }
}

