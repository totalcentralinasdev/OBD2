#ifndef PLOTPRESENTER_HPP
#define PLOTPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class plotView;

class plotPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    plotPresenter(plotView& v);

    /**
     * The activate function is called automatically when this screen is "switched in"
     * (ie. made active). Initialization logic can be placed here.
     */
    virtual void activate();

    /**
     * The deactivate function is called automatically when this screen is "switched out"
     * (ie. made inactive). Teardown functionality can be placed here.
     */
    virtual void deactivate();

    virtual ~plotPresenter() {}

private:
    plotPresenter();

    plotView& view;
};

#endif // PLOTPRESENTER_HPP
