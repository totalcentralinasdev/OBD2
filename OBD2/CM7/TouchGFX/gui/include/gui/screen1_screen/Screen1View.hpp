#ifndef SCREEN1VIEW_HPP
#define SCREEN1VIEW_HPP

#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

class Screen1View : public Screen1ViewBase
{
public:
    Screen1View();
    virtual ~Screen1View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void PIN1_exe();
    virtual void PIN3_exe();
    virtual void PIN6_exe();
    virtual void PIN7_exe();
    virtual void PIN8_exe();
    virtual void PIN9_exe();
    virtual void PIN11_exe();
    virtual void PIN12_exe();
	virtual void PIN13_exe();
	virtual void PIN14_exe();
	virtual void PIN15_exe();
	virtual void PIN16_exe();
	virtual void update_values_exe();

	void boxWithBorder1ClickHandler(const BoxWithBorder& b, const ClickEvent& e);
	void Resistor_icon_ClickHandler(const ScalableImage& b, const ClickEvent& e);

protected:
	Callback<Screen1View, const BoxWithBorder&, const ClickEvent&> boxWithBorder1ClickedCallback;
	Callback<Screen1View, const ScalableImage&, const ClickEvent&> Termination_Resistor_Callback;
};

#endif // SCREEN1VIEW_HPP
