/**
* \author Xuan Zhang
* \data Mai-July 2018
*/
#ifndef SEMANTICS_MAX_H
#define SEMANTICS_MAX_H

#include <octomap/ColorOcTree.h>

namespace octomap
{

  /// Structure contains semantic colors and their confidences
  struct SemanticsMax
  {
    ColorOcTreeNode::Color semantic_color; ///<Semantic color
    float confidence;

    SemanticsMax():semantic_color(), confidence(0.){}

    bool operator==(const SemanticsMax& rhs) const
    {
        return semantic_color == rhs.semantic_color
                && confidence == rhs.confidence;
    }

    bool operator!=(const SemanticsMax& rhs) const
    {
        return !(*this == rhs);
    }

    ColorOcTreeNode::Color getSemanticColor() const
    {
      return semantic_color;
    }

    bool isSemanticsSet() const
    {
      if(semantic_color != ColorOcTreeNode::Color(255,255,255))
        return true;
      return false;
    }

    /// Perform max fusion
    static SemanticsMax semanticFusion(const SemanticsMax s1, const SemanticsMax s2)
    {
      // std::cout << "Max semantic Fusion..." << std::endl;
      SemanticsMax ret;
      // If the same color, update the confidence to the average
      if(s1.semantic_color == s2.semantic_color)
      {
        ret.semantic_color = s1.semantic_color;
        ret.confidence = (s1.confidence + s2.confidence) / 2.;
      }
      // If color is different, keep the larger one and drop a little for the disagreement
      else
      {
        // std::cout << "s1.confidence: " << s1.confidence << std::endl;
        // std::cout << "s2.confidence: " << s2.confidence << std::endl;
        // ret = s1.confidence > s2.confidence ? s1 : s2;
        ret = s2;
        ret.confidence *= 0.9;
      }
      return ret;
      // return s1;
    }
  };

  std::ostream& operator<<(std::ostream& out, SemanticsMax const& s);
}
#endif //SEMANTICS_MAX_H
