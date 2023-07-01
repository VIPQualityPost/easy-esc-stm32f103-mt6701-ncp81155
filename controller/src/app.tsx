import { createRoot } from "react-dom/client";
import { Home } from "./Home";

function render() {
  const root = createRoot(document.getElementById("app"));
  root.render(<Home />);
}

render();
