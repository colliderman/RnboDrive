/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout()
{
    juce::AudioProcessorValueTreeState::ParameterLayout layout;
    
    layout.add(std::make_unique<juce::AudioParameterFloat> (juce::ParameterID {"drive", 1}, "Drive", juce::NormalisableRange<float>(1.0f, 10.0f, 0.1f, 1.0f), 5.0f));
    layout.add(std::make_unique<juce::AudioParameterFloat> (juce::ParameterID {"output", 1}, "Output", juce::NormalisableRange<float>(0.0f, 127.0f, 0.1f, 1.0f), 127.0f));
    layout.add(std::make_unique<juce::AudioParameterFloat> (juce::ParameterID {"mix", 1}, "Mix", juce::NormalisableRange<float>(0.0f, 1.0f, 0.0f, 1.0f), 1.0f));

    return layout;
}

//==============================================================================
RnboDriveAudioProcessor::RnboDriveAudioProcessor()
#ifndef JucePlugin_PreferredChannelConfigurations
     : AudioProcessor (BusesProperties()
                     #if ! JucePlugin_IsMidiEffect
                      #if ! JucePlugin_IsSynth
                       .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                      #endif
                       .withOutput ("Output", juce::AudioChannelSet::stereo(), true)
                     #endif
                       ),
#endif
    vt (*this, nullptr, "Parameters", createParameterLayout())
{
    for (RNBO::ParameterIndex i = 0; i < rnboObj.getNumParameters(); ++i)
        {
            RNBO::ParameterInfo info;
            rnboObj.getParameterInfo (i, &info);

            if (info.visible)
            {
                auto paramID = juce::String (rnboObj.getParameterId (i));

                // Each apvts parameter id and range must be the same as the rnbo param object's.
                // If you hit this assertion then you need to fix the incorrect id in ParamIDs.h.
                jassert (vt.getParameter (paramID) != nullptr);

                // If you hit these assertions then you need to fix the incorrect apvts
                // parameter range in createParameterLayout().
                jassert (info.min == vt.getParameterRange (paramID).start);
                jassert (info.max == vt.getParameterRange (paramID).end);

                apvtsParamIdToRnboParamIndex[paramID] = i;

                vt.addParameterListener (paramID, this);
                rnboObj.setParameterValue (i, vt.getRawParameterValue (paramID)->load());
            }
        }
}

RnboDriveAudioProcessor::~RnboDriveAudioProcessor()
{
}

//==============================================================================
const juce::String RnboDriveAudioProcessor::getName() const
{
    return JucePlugin_Name;
}

bool RnboDriveAudioProcessor::acceptsMidi() const
{
   #if JucePlugin_WantsMidiInput
    return true;
   #else
    return false;
   #endif
}

bool RnboDriveAudioProcessor::producesMidi() const
{
   #if JucePlugin_ProducesMidiOutput
    return true;
   #else
    return false;
   #endif
}

bool RnboDriveAudioProcessor::isMidiEffect() const
{
   #if JucePlugin_IsMidiEffect
    return true;
   #else
    return false;
   #endif
}

double RnboDriveAudioProcessor::getTailLengthSeconds() const
{
    return 0.0;
}

int RnboDriveAudioProcessor::getNumPrograms()
{
    return 1;   // NB: some hosts don't cope very well if you tell them there are 0 programs,
                // so this should be at least 1, even if you're not really implementing programs.
}

int RnboDriveAudioProcessor::getCurrentProgram()
{
    return 0;
}

void RnboDriveAudioProcessor::setCurrentProgram (int index)
{
}

const juce::String RnboDriveAudioProcessor::getProgramName (int index)
{
    return {};
}

void RnboDriveAudioProcessor::changeProgramName (int index, const juce::String& newName)
{
}

//==============================================================================
void RnboDriveAudioProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    // Use this method as the place to do any pre-playback
    // initialisation that you need..
    rnboObj.prepareToProcess(sampleRate, static_cast<size_t>(samplesPerBlock));
}

void RnboDriveAudioProcessor::releaseResources()
{
    // When playback stops, you can use this as an opportunity to free up any
    // spare memory, etc.
}

#ifndef JucePlugin_PreferredChannelConfigurations
bool RnboDriveAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
  #if JucePlugin_IsMidiEffect
    juce::ignoreUnused (layouts);
    return true;
  #else
    // This is the place where you check if the layout is supported.
    // In this template code we only support mono or stereo.
    // Some plugin hosts, such as certain GarageBand versions, will only
    // load plugins that support stereo bus layouts.
    if (layouts.getMainOutputChannelSet() != juce::AudioChannelSet::mono()
     && layouts.getMainOutputChannelSet() != juce::AudioChannelSet::stereo())
        return false;

    // This checks if the input layout matches the output layout
   #if ! JucePlugin_IsSynth
    if (layouts.getMainOutputChannelSet() != layouts.getMainInputChannelSet())
        return false;
   #endif

    return true;
  #endif
}
#endif

void RnboDriveAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer& midiMessages)
{
    rnboObj.process(buffer.getArrayOfReadPointers(),
                    static_cast<RNBO::Index>(buffer.getNumChannels()),
                    buffer.getArrayOfWritePointers(),
                    static_cast<RNBO::Index>(buffer.getNumChannels()),
                    static_cast<RNBO::Index>(buffer.getNumSamples()));
}

//==============================================================================
bool RnboDriveAudioProcessor::hasEditor() const
{
    return true; // (change this to false if you choose to not supply an editor)
}

juce::AudioProcessorEditor* RnboDriveAudioProcessor::createEditor()
{
    //return new RnboDriveAudioProcessorEditor (*this);
    return new juce::GenericAudioProcessorEditor (*this);
}

//==============================================================================
void RnboDriveAudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    // You should use this method to store your parameters in the memory block.
    // You could do that either as raw data, or use the XML or ValueTree classes
    // as intermediaries to make it easy to save and load complex data.
}

void RnboDriveAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    // You should use this method to restore your parameters from this memory block,
    // whose contents will have been created by the getStateInformation() call.
}

void RnboDriveAudioProcessor::parameterChanged(const juce::String& parameterID, float newValue)
{
    rnboObj.setParameterValue (apvtsParamIdToRnboParamIndex[parameterID], newValue);
}
//==============================================================================
// This creates new instances of the plugin..
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new RnboDriveAudioProcessor();
}
