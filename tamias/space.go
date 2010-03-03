package tamias


// Number of frames that contact information should persist.
var CONTACT_PERSISTENCE = 6

// User collision handler function types.
type CollisionFunc func(arb * Arbiter, space * Space, data interface{}) (int)

// Structure for holding collision pair function information.
// Used internally.
type CollisionHandler struct {
  a, b CollisionType  
  begin, preSolve, postSolve, separate CollisionFunc
  data interface {}
}

const CP_MAX_CONTACTS_PER_ARBITER = 6

type ContactBufferHeader struct {
  stamp       int
  next        * ContactBufferHeader
  numContacts uint
} 


type Space struct {
  // *** User definable fields  
  // Number of iterations to use in the impulse solver to solve contacts.
  Iterations int
  
  // Number of iterations to use in the impulse solver to solve elastic collisions.
  ElasticIterations int
  
  // Default gravity to supply when integrating rigid body motions.
  Gravity Vect
  
  // Default damping to supply when integrating rigid body motions.
  Damping Float
  
  // *** Internally Used Fields  
  // When the space is locked, you should not add or remove objects;  
  locked int
  
  // Time stamp. Is incremented on every call to cpSpaceStep().
  stamp int

  // The static and active shape spatial hashes.
  staticShapes SpaceHash
  activeShapes SpaceHash
  
  // List of bodies in the system.
  bodies *Array
  
  // List of active arbiters for the impulse solver.
  arbiters, pooledArbiters *Array; 
  
  // Linked list ring of contact buffers.
  // Head is the current buffer. Tail is the oldest buffer.
  // The list points in the direction of tail->head.
  contactBuffersHead, contactBuffersTail *ContactBufferHeader;
  
  // List of buffers to be free()ed when destroying the space.
  // Not needed in Go
  // cpArray *allocatedBuffers;
  
  // Persistant contact set.
  contactSet *HashSet;
  
  // List of constraints in the system.
  constraints *Array;
  
  // Set of collisionpair functions.
  collFuncSet *HashSet;
  // Default collision handler.
  defaultHandler CollisionHandler;
    
  postStepCallbacks *HashSet;  
  
} 


