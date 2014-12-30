#include "MiniConfig.h"
#include <cinder/xml.h>
#include <cinder/Utilities.h>
#include <cinder/app/App.h>
#include <cinder/params/Params.h>

using namespace ci;
using namespace ci::app;
using namespace std;

#define GROUP_DEF(grp)
#define ITEM_DEF(type, var, default) type var = default;
#define ITEM_DEF_MINMAX(type, var, default, Min, Max) ITEM_DEF(type, var, default);
#include "item.def"
#undef ITEM_DEF_MINMAX
#undef ITEM_DEF
#undef GROUP_DEF

namespace
{
    const string kConfigFileName = "MiniConfig.xml";

    void revertToDefaultValues()
    {
#define GROUP_DEF(grp)
#define ITEM_DEF(type, var, default) var = default;
#define ITEM_DEF_MINMAX(type, var, default, Min, Max) ITEM_DEF(type, var, default);
#include "item.def"
#undef ITEM_DEF_MINMAX
#undef ITEM_DEF
#undef GROUP_DEF
        console() << "MiniConfig reverted to default values" << endl;
    }
}

void readConfig()
{
	fs::path configPath = getAssetPath("./") / kConfigFileName;
	try
	{
		XmlTree tree(loadFile(configPath));
        XmlTree group;

#define GROUP_DEF(grp) group = tree.getChild(#grp);
#define ITEM_DEF(type, var, default)                        \
    do                                                      \
    {                                                       \
        if (group.getTag().empty())                         \
            var = tree.getChild(#var).getValue<type>();     \
        else                                                \
            var = group.getChild(#var).getValue<type>();    \
    } while (0);
#define ITEM_DEF_MINMAX(type, var, default, Min, Max) ITEM_DEF(type, var, default);
#include "item.def"
#undef ITEM_DEF_MINMAX
#undef ITEM_DEF
#undef GROUP_DEF
        console() << "Reads from " << configPath.string() << endl;
	}
	catch (exception& e)
    {
        console() << e.what() << endl;
		console() << "[Warning] Fails to read from " << configPath.string() << endl;
        revertToDefaultValues();
		writeConfig();
	}
}

void writeConfig()
{
	fs::path configPath = getAssetPath("./") / kConfigFileName;
	try
	{
        XmlTree tree = XmlTree::createDoc();
        XmlTree group;

#define GROUP_DEF(grp)                                          \
        do                                                      \
        {                                                       \
            if (!group.getTag().empty()) tree.push_back(group); \
            group = XmlTree(#grp, "");                          \
        } while (0);

#define ITEM_DEF(type, var, default)                            \
        do                                                      \
        {                                                       \
            XmlTree item(#var, toString(var));                  \
            if (group.getTag().empty()) tree.push_back(item);   \
            else group.push_back(item);                         \
        } while (0);
#define ITEM_DEF_MINMAX(type, var, default, Min, Max) ITEM_DEF(type, var, default);
#include "item.def"
#undef ITEM_DEF_MINMAX
#undef ITEM_DEF
#undef GROUP_DEF
        if (!group.getTag().empty()) tree.push_back(group);

#ifdef CHINESE_GBK_ENCODING_ENABLED
        // OStreamRef os = writeFile(configPath)->getStream();
        ofstream of(configPath.string().c_str());
        const string kGbkHeader = "<?xml version=\"1.0\" encoding=\"gbk\"?>";
        of << kGbkHeader << endl << tree;
#else
        tree.write( writeFile(configPath));
#endif
		console() << "Writes to " << configPath.string() <<endl;
	}
	catch( ... ) {
		console() << "[Warning] Fails to write to " << configPath.string() <<endl;
	}
}

void setupConfigUI(cinder::params::InterfaceGl* params)
{
    params->addButton("SAVE", writeConfig);
#define GROUP_DEF(grp)                  params->addSeparator(#grp);       
#define ITEM_DEF(type, var, default)    params->addParam(#var, &var);
#define ITEM_DEF_MINMAX(type, var, default, Min, Max)               \
    do                                                              \
    {                                                               \
        type step = (Max - Min) / (type)100;                        \
        params->addParam(#var, &var).min(Min).max(Max).step(step);  \
    } while(0);
#include "item.def"
#undef ITEM_DEF_MINMAX
#undef ITEM_DEF
#undef GROUP_DEF
    params->addSeparator();
}

namespace
{
    const int kPODItemHeight = 16;
    template <typename T>
    int getItemHeight(const T&)
    {
        return kPODItemHeight;
    }

    template <>
    int getItemHeight(const quat&)
    {
        return 80;
    }

    template <>
    int getItemHeight(const vec3&)
    {
        return 120;
    }

    template <>
    int getItemHeight(const Color&)
    {
        return 80;
    }

    template <>
    int getItemHeight(const ColorA&)
    {
        return 80;
    }
}

int getConfigUIHeight()
{
    int height = kPODItemHeight * 4; // top + bottom

#define GROUP_DEF(grp)                  height += kPODItemHeight;
#define ITEM_DEF(type, var, default)    height += getItemHeight(var);
#define ITEM_DEF_MINMAX(type, var, default, Min, Max) ITEM_DEF(type, var, default);
#include "item.def"
#undef ITEM_DEF_MINMAX
#undef ITEM_DEF
#undef GROUP_DEF

    return height;
}
