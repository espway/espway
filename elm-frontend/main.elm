import Browser
import Url
import Debug exposing (log)
import Bitwise exposing (and)
import Browser.Navigation as Nav
import Browser.Events exposing (onMouseMove, onMouseUp)
import Html exposing (Html, button, div, text, a, br)
import Html.Attributes exposing (href)
import Html.Events exposing (onClick, custom, on)
import Svg exposing (svg, circle)
import Svg.Attributes exposing (cx, cy, r, width, height, viewBox)
import Json.Decode as D

main : Program () Model Msg
main =
  Browser.application
    { init = init
    , update = update
    , view = view
    , subscriptions = subscriptions
    , onUrlChange = onUrlChange
    , onUrlRequest = onUrlRequest
    }

type Msg = MouseMove Float Float
         | MouseUp
         | MsgNone

type MyView = First | Second

type JoystickPosition = Free
                      | Position Float Float

type alias Model =
  { key : Nav.Key
  , theview : MyView
  , joystickPosition : JoystickPosition
  }

init : () -> Url.Url -> Nav.Key -> ( Model, Cmd Msg )
init _ _ key = ( Model key First Free, Cmd.none )

onUrlChange : Url.Url -> Msg
onUrlChange _ = MsgNone

onUrlRequest : Browser.UrlRequest -> Msg
onUrlRequest _ = MsgNone

decodeTouchMove = 
  D.map4 (\x y offX offY -> MouseMove (x - offX) (y - offY))
    (D.at ["touches", "0", "pageX"] D.float)
    (D.at ["touches", "0", "pageY"] D.float)
    (D.at ["currentTarget", "offsetLeft"] D.float)
    (D.at ["currentTarget", "offsetTop"] D.float)

decodeMouseMove = 
  D.field "buttons" D.int
  |> D.andThen (\btns -> if (btns |> and 1) /= 0 then decodeMouseCoords
                                                 else D.succeed MsgNone)

decodeMouseCoords =
  D.map4 (\x y offX offY -> MouseMove (x - offX) (y - offY))
    (D.field "pageX" D.float)
    (D.field "pageY" D.float)
    (D.at ["currentTarget", "offsetLeft"] D.float)
    (D.at ["currentTarget", "offsetTop"] D.float)

subscriptions model = 
  Sub.batch
    [ onMouseUp <| D.succeed MouseUp ]

updateModel msg model = 
  case msg of
    MouseUp -> { model | joystickPosition = Free }
    MouseMove x y -> { model | joystickPosition = Position x y }
    MsgNone -> model

update : Msg -> Model -> ( Model, Cmd Msg )
update msg model = ( updateModel msg model, Cmd.none )

navbar = div [] [ a [ href "/first" ] [ text "First view " ]
                , a [ href "/second" ] [ text "Second view" ]
                ]

preventDefaultAndStopPropagation msg =
  { message = msg
  , stopPropagation = True
  , preventDefault = True }

joystick model =
  let
    (x, y) = case model.joystickPosition of
      Free -> (160, 160)
      Position px py -> (px, py)
    w = String.fromFloat 320
    h = String.fromFloat 320
  in
  div [ custom "mousemove" <| D.map preventDefaultAndStopPropagation decodeMouseMove
      , custom "touchmove" <| D.map preventDefaultAndStopPropagation decodeTouchMove
      , custom "touchend" <| D.map preventDefaultAndStopPropagation <| D.succeed MouseUp
      ]
    [
      svg
        [ width w
        , height h
        , viewBox <| "0 0 " ++ w ++ " " ++ h
        ]
        [
          circle
            [ cx <| String.fromFloat x
            , cy <| String.fromFloat y
            , r "10"
            ]
            []
        ]
    ]

view : Model -> Browser.Document Msg
view model =
  { title = "My app"
  , body = 
    [ navbar
    , joystick model
    ]
  }